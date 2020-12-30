/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
#include "includes/linkState.h"
#include "includes/routingTable.h"
#include "includes/socket.h"
#include "includes/packetTCP.h"

module Node{ //each node uses these interfaces add interfaces with <uses interface>
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;

   uses interface Hashmap<uint32_t> as neighbors;
   uses interface Hashmap<uint32_t> as activeNodes;
   uses interface Hashmap<uint32_t> as packets;
   uses interface Hashmap<uint32_t> as updates;

   uses interface List<int> as cost;
  uses interface List<socket_store_t> as sockets;

   uses interface Timer<TMilli> as ntimer; //gets each node to use the timer instance
   uses interface Timer<TMilli> as tTimer;

   //uses interface routingTable as routing; //offline. not working

}

implementation{ //code that each node runs | implementation of node module
   pack sendPackage;
   routingTable rT;
   linkState linkState = {0};
   int linkCost;
   uint8_t curSeq = 0;
   bool activeSockets[MAX_NUM_OF_SOCKETS];


   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);

   //flood + neighbor discovery
   bool checkPackage(nx_uint16_t src, nx_uint16_t seq);
   void addNeighbor(nx_uint16_t node);

   //routingTable
   void updateRT();
   void protocoltwo(); //broadcast neighbors to other nodes
   void updateLS(uint16_t src, uint16_t dest, uint16_t seq, int dist);
   bool checkLS(uint16_t src, uint16_t dest, uint16_t seq, int dist);
   void checkTS(); //see if node is alive
   int hopCount(int dest);
   int minDist();

   //TCP Proj 3
   void TCPHandler(pack* myMsg);
   void TCPdata(pack* myMsg);
   void closeSockets();
   uint8_t findClosedSocket();
   socket_store_t* getSocket(nx_uint16_t port); //find socket with correct src and dest
   socket_store_t* getSocketAddress(nx_uint16_t addr); //find listening socket to open connection

   //Proj 4
   void ClientHandler(pack* myMsg);
   void ClientRequest(pack* myMsg);



   event void Boot.booted(){ //each node starts when boot event is activated [activated by python end]

      call AMControl.start(); //calls AMControl | be called <call>
      call ntimer.startPeriodic(4000); //time in miliseconds

      dbg(GENERAL_CHANNEL, "Booted\n");

   }

   //once called node starts "listening"
   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }


   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){ //event a node recieves a message. This code runs
    //  dbg(GENERAL_CHANNEL, "Packet Received\n");

      if(len==sizeof(pack)){
         pack* myMsg=(pack*) payload;
      //   dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);
      if(myMsg -> dest != 0){


        if(myMsg->protocol != 2){
          //debug statements
          dbg(FLOODING_CHANNEL, "Package Sent. Sending from: %d to %d\n", myMsg->src, myMsg->dest);
          dbg(FLOODING_CHANNEL, "TTL: %d\n", myMsg->TTL);

          //check if package is at destination
          if((myMsg->dest == TOS_NODE_ID) && !(checkPackage(myMsg ->src, myMsg ->seq))){
          //  dbg(ROUTING_CHANNEL, "Packet destination reached. Payload: %s \n", myMsg->payload);
            if(myMsg ->protocol == 0){
              //dbg(ROUTING_CHANNEL, "Reply Ping\n");
              makePack(&sendPackage, TOS_NODE_ID, myMsg ->src, 19,1,(sendPackage.seq+1),"package here",PACKET_MAX_PAYLOAD_SIZE);
              call Sender.send(sendPackage, hopCount(myMsg->src));
              return msg;
            }else if(myMsg->protocol == 4){//proj 3
                  dbg(GENERAL_CHANNEL,"Got my TCP packet!\n");
                  TCPHandler(myMsg);
            }else if(myMsg->protocol == 5){//proj 3
                dbg(GENERAL_CHANNEL,"DATA TCP packet!\n");
                TCPdata(myMsg);
            }else if(myMsg->protocol == 6){//proj 4
                ClientHandler(myMsg);
            }else if(myMsg->protocol ==7){//proj 4
                ClientRequest(myMsg);
            }
          }else if(myMsg->TTL > 0 && !(checkPackage(myMsg->src, myMsg->seq)) && (myMsg -> src != TOS_NODE_ID)){ //Repackage and continue flooding
                  dbg(FLOODING_CHANNEL, "Flooding Network... \n");
                  makePack(&sendPackage, myMsg->src, myMsg->dest, (myMsg->TTL - 1), myMsg -> protocol, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);//protocol 0 = ping
                  //Flood the network
                  call Sender.send(sendPackage, AM_BROADCAST_ADDR);
                  return msg;
            }else{
              dbg(FLOODING_CHANNEL, "Duplicate Packet\n");
              return msg;
            }
        }else{
            //dbg(GENERAL_CHANNEL,"protocol 2\n");
            linkCost = atoi(myMsg -> payload); //turn payload string to cost int

            //check for linkstate updates if update needed, excecute linkStateUpdate
            if(checkLS(myMsg -> src, myMsg -> dest, myMsg -> seq, linkCost)){

              updateLS(myMsg -> src, myMsg -> dest, myMsg -> seq, linkCost);

              if(myMsg -> TTL > 0){
                makePack(&sendPackage, myMsg->src, myMsg->dest, (myMsg->TTL - 1), 2, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);
                call Sender.send(sendPackage, AM_BROADCAST_ADDR);
              }
            }
        }
      }
        //For neighbor discovery | check and add neighbor if applicable
        if(myMsg -> protocol != 2){
          addNeighbor(myMsg -> src);
        }
        return msg;
      }
      dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
      return msg;
   }

   //create a package and broadcast to all nodes when timer fires
   event void ntimer.fired(){
     uint32_t timeStamp = call ntimer.getNow();
     makePack(&sendPackage, TOS_NODE_ID, 0, 0, 0, (sendPackage.seq + 1), "Neighbor", PACKET_MAX_PAYLOAD_SIZE);
     call Sender.send(sendPackage, AM_BROADCAST_ADDR);

     if(timeStamp > 12000){
       protocoltwo(); //boradcast neighbors
     }
     checkTS();
   }

   event void tTimer.fired(){ //tcp timer for possible dropped packets
    uint8_t i,j,k;//iterators
    socket_store_t* socket;
    DATA_PACK dataPack;
    uint8_t remaining = socket->totalBytes - socket->bytesSent;

    for(i=0; i < MAX_NUM_OF_SOCKETS; i++){
        if(activeSockets[i]!=FALSE){
          socket = getSocket(i);
          if(socket->state == ESTABLISHED){
            for(j = 0; j<WINDOWSIZE; j++){
              if(socket->outstandPacks[j] ==0 ){
                for(k = 0; k<DATA_SIZE; k++){
                  dataPack.arr[k] = (uint8_t)'A';
                }
                dataPack.arr[0]=socket->seqMsg;
                dataPack.size = DATA_SIZE;
                socket->seqMsg = socket->seqMsg + (DATA_SIZE-1);

                dataPack.port = socket->dest.port; //MAKE FUNCITON FOR DATA PACKS
                makePack(&sendPackage, TOS_NODE_ID, socket->dest.addr, 18, 5, curSeq,(uint8_t*)&dataPack, PACKET_MAX_PAYLOAD_SIZE);

                socket->outstandPacks[j] = curSeq;
                socket->nonAckPack[j] = sendPackage;
                socket->resend[j] = 2;
                curSeq++;
                call Sender.send(sendPackage, hopCount(socket->dest.addr));
              }else{

                socket->resend[j] = socket->resend[j]-1;
                if(socket->resend[j] == 0 || socket->resend[j] < 0){
                  socket->resend[j] = 2;
                  sendPackage = socket->nonAckPack[j];
                  call Sender.send(sendPackage, hopCount(socket->dest.addr));
                }

              }
            }
          }
        }
    }
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 18, 0, (sendPackage.seq+1), payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, hopCount(destination));
   }

   event void CommandHandler.printNeighbors(){
     int i = 0;
     uint16_t j= call neighbors.size();
     uint32_t* node = call neighbors.getKeys();
     printf(">-------->PRINTING NEIGHBORS FOR NODE %d<--------<\n", TOS_NODE_ID);

     for(i; i < j; i++){
       printf("Neighbor (%d) is: %d\n", i, node[i]);
     }
   }

   event void CommandHandler.printRouteTable(){
     int i;
     printf(">-------->ROUTING TABLE FOR NODE %d<--------<\n", TOS_NODE_ID);
     printf("Node\tCost\tHop\n");
     for(i = 0; i < 500 ; i++){
       if(rT[i].cost < 500){
         printf("%d\t%d\t%d\n", i, rT[i].cost, rT[i].hops);
       }
     }

   }

   event void CommandHandler.printLinkState(){
     int* currentNodes = call activeNodes.getKeys();
     int counter = call activeNodes.size();
     int neighborNodes;
     int currentNode;
     int i;
     int j;
     printf(">-------->Link State<--------<\n");

     for(i = 0; i < counter; i++){
       currentNode = currentNodes[i];
       for(j = 0; j < counter; j++){
         neighborNodes = currentNodes[j];
         if(linkState[currentNode].cost[neighborNodes] != 0){
           printf("Node %d and Node %d are connected \n", currentNode, neighborNodes);
         }
       }
     }

   }

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(uint8_t  port){
     socket_store_t* socket;
     uint8_t openSocket = findClosedSocket();

     dbg(TRANSPORT_CHANNEL,"Initializing socket %d for port %d\n", openSocket, port);
     socket = call sockets.getAddress(openSocket);
     socket->src.port = port;
     socket->lastRead = 0;
   }

   event void CommandHandler.setTestClient(uint16_t destination, uint8_t srcPort, uint8_t destPort, uint8_t data){
    uint16_t i;
    tcp_packet myTCP;
    socket_store_t* socket; //pointer directly modifies socket in list
    uint8_t openSocket = findClosedSocket();
    socket = call sockets.getAddress(openSocket);
    activeSockets[srcPort] = TRUE;

    socket->dest.addr = destination;
    socket->dest.port = destPort;
    socket->src.port = srcPort;
    socket->totalBytes = data;
    socket->bytesSent = 0;
    socket->seqMsg = 0;

    for(i = 0; i < data; i++){
      socket->sendBuff[i] = (uint8_t)'a';
    }
    for(i = 0; i < WINDOWSIZE; i++){
      socket->outstandPacks[i] = 0;
      socket->nonAckPack[i] = sendPackage;
    }

    myTCP.destPort = destPort;
    myTCP.srcPort = srcPort;
    myTCP.flag = SYN_FLAG;


    makePack(&sendPackage, TOS_NODE_ID, destination, 18, 4, curSeq, (uint8_t*)&myTCP, sizeof(tcp_packet));//start TCP with myself
    call Sender.send(sendPackage, hopCount(destination));
    socket->state = SYN_SENT;
    socket->lastSent = curSeq;
    curSeq++;

    call tTimer.startPeriodic(50000);
   }

   event void CommandHandler.endConnection(uint16_t destination, uint8_t srcPort, uint8_t destPort){
     tcp_packet myTCP;
     socket_store_t* socket;

     dbg(TRANSPORT_CHANNEL,"Closing Connection\n");
     socket = getSocket(srcPort);
     activeSockets[socket->src.port] = FALSE;
     myTCP.srcPort = socket -> src.port;
     myTCP.destPort = destPort;
     myTCP.flag = FIN_FLAG;

     makePack(&sendPackage, TOS_NODE_ID, destination, 18, 4, curSeq, (uint8_t*)&myTCP, PACKET_MAX_PAYLOAD_SIZE);
     call Sender.send(sendPackage, hopCount(destination));
     curSeq++;
     socket->state = CLOSED;
   }

   event void CommandHandler.setAppServer(uint8_t  port){
     socket_store_t* socket;
     uint8_t openSocket = findClosedSocket();

     dbg(TRANSPORT_CHANNEL,"Node %d on port %d\n", TOS_NODE_ID, port);
     socket = call sockets.getAddress(openSocket);
     socket->numClients = 0;
     socket->src.port = port;
     socket->state = LISTEN;
     socket->lastRead = 0;

   }

   event void CommandHandler.setAppClient(uint16_t destination, uint8_t srcPort, uint8_t destPort, uint8_t size, uint8_t* data){
     uint8_t i;
     socket_store_t* socket;
     DATA_PACK dataP;
     uint8_t openSocket = findClosedSocket();
     socket = call sockets.getAddress(openSocket);
     activeSockets[srcPort] = TRUE;

     dbg(TRANSPORT_CHANNEL,"Setting Up Client...\n");

     socket->src.port = srcPort;
     socket->dest.addr = destination;
     socket->dest.port = destPort;
     socket->totalBytes = data;
     socket->bytesSent = 0;
     socket->seqMsg = 0;

     dbg(TRANSPORT_CHANNEL,"Adding Client: %c%c%c\n", data[0],data[1],data[2]);
     dataP.size = size;
     dataP.port = destPort;
     dataP.arr[0] = srcPort;

     for(i = 1; i < size; i++){
       dataP.arr[i] = data[i-1];
     }

     makePack(&sendPackage, TOS_NODE_ID, destination, 18, 6, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
     call Sender.send(sendPackage, hopCount(destination));
     socket->state = SYN_SENT;
     socket->lastSent = curSeq;
     curSeq++;
   }

   event void CommandHandler.clientMessage(uint8_t srcPort, uint8_t usrname, uint8_t size, uint8_t* data){
     uint8_t i,os;
     DATA_PACK dataP;
     dataP.port = 41;
     dataP.arr[0] = (uint8_t)'W';
     dataP.arr[1] = usrname;
     dataP.size = size;
     os = size+2;

     dbg(TRANSPORT_CHANNEL,"Prepping Message...\n");
     for(i = 0; i < size; i++){
       dataP.arr[i+2] = data[i];
     }
     for(i = 0; i <usrname; i++){
       dataP.arr[i+os] = data[i+size];
     }
     makePack(&sendPackage, TOS_NODE_ID, 1, 18, 7, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
     call Sender.send(sendPackage, hopCount(1));
     dbg(TRANSPORT_CHANNEL,"Message Sent\n");
     curSeq++;

   }

   event void CommandHandler.broadcast(uint8_t srcPort, uint8_t size, uint8_t* data){
     uint8_t i;
     DATA_PACK dataP;
     dataP.port = 41;
     dataP.size = srcPort;
     dataP.arr[0] = (uint8_t)'B';

     dbg(TRANSPORT_CHANNEL,"Prepping Broadcast...\n");
     for(i = 0; i < size; i++){
       dataP.arr[i+1] = data[i];
     }
     makePack(&sendPackage, TOS_NODE_ID, 1, 18, 7, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
     call Sender.send(sendPackage, hopCount(1));
     dbg(TRANSPORT_CHANNEL,"Broadcast Sent\n");
     curSeq++;

   }

   event void CommandHandler.request(uint8_t srcPort){
     DATA_PACK data;

     data.port = 41;
     data.size = srcPort;
     data.arr[0] = (uint8_t)'R';
     dbg(TRANSPORT_CHANNEL,"Prepping Request...\n");
     makePack(&sendPackage, TOS_NODE_ID, 1, 18, 7, curSeq, (uint8_t*)&data, sizeof(DATA_PACK));
     call Sender.send(sendPackage, hopCount(1));
     dbg(TRANSPORT_CHANNEL,"Request Sent...\n");
     curSeq++;
   }




   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }

   void addNeighbor(nx_uint16_t node){
     uint32_t timeStamp = call ntimer.getNow();

     if(call neighbors.contains(node) == 0){
       call neighbors.insert(node, timeStamp);
     }else{
       call neighbors.remove(node);
       call neighbors.insert(node, timeStamp);
     }

     //node is back online
     if(call neighbors.contains(node) == 0 && timeStamp > 4000*5){
        dbg(NEIGHBOR_CHANNEL, "Neighbor %d is online", node);

        updateLS(node, TOS_NODE_ID, (sendPackage.seq + 1), "1");
        //broadcast online node
        makePack(&sendPackage, TOS_NODE_ID, node, 18, 2, (sendPackage.seq + 1), "1", PACKET_MAX_PAYLOAD_SIZE);
        call Sender.send(sendPackage, AM_BROADCAST_ADDR);
     }
   }

    bool checkPackage(nx_uint16_t src, nx_uint16_t seq){
       nx_uint16_t hashSeq;
       if(call packets.contains(src) == 1){ //source has been seen before
         hashSeq = call packets.get(src);
         if(hashSeq > seq){ //packet is a duplicate
           return TRUE;
         }else{ //packet is not duplicate, reset seq
           call packets.remove(src);
           call packets.insert(src, seq);
           return FALSE;
         }
       }else{ //source has not been seen before
         call packets.insert(src,seq);
         return FALSE;
       }
   }

   int hopCount(int dest){
     return rT[dest].hops;
   }


   void protocoltwo(){
     int* neighborNode = call neighbors.getKeys();
     int counter = call neighbors.size();
     int i = 0;

     for(i; i < counter; i++){
       makePack(&sendPackage, TOS_NODE_ID, neighborNode[i], 18, 2, (sendPackage.seq + 1), "1", PACKET_MAX_PAYLOAD_SIZE); //node is online
       call Sender.send(sendPackage, AM_BROADCAST_ADDR);
     }
     checkTS();
   }

   void checkTS(){
     int i;
     int neighborIndex = call neighbors.size();
     int* neighborKeys = call neighbors.getKeys();
     uint32_t currentTS = call ntimer.getNow();
     uint32_t nodeTS;

     for(i = 0; i < neighborIndex; i++){//remove neighbor if not online
       nodeTS = call neighbors.get(neighborKeys[i]);

       if((currentTS - nodeTS) > 12000){
         dbg(NEIGHBOR_CHANNEL, "Neighbor %d is offline\n", neighborKeys[i]);

         //message about offline neighbor
         makePack(&sendPackage, TOS_NODE_ID,neighborKeys[i], 18, 2, (sendPackage.seq+1), "0", PACKET_MAX_PAYLOAD_SIZE);
         call Sender.send(sendPackage, AM_BROADCAST_ADDR);

         updateLS(TOS_NODE_ID, neighborKeys[i], (sendPackage.seq + 1), "0");
       }//neighbor is online, do nothing
     }
   }

   bool checkLS(uint16_t src, uint16_t dest, uint16_t seq, int dist){
     if(call updates.contains(src) == 1){
       if(call updates.get(src) < seq){
         return TRUE;
       }else{
         return FALSE;
       }
     }

     return TRUE;
   }

   void updateLS(uint16_t src, uint16_t dest, uint16_t seq, int dist){
      linkState[src].cost[dest] = dist;

      if(call activeNodes.contains(dest) == 1){
        call activeNodes.remove(dest);
      }
      if(call updates.contains(src)==1){
        call updates.remove(src);
      }

      if(dist == 1){
        dbg(NEIGHBOR_CHANNEL,"Node %d is now active. Adding to Link State\n", dest);
        call activeNodes.insert(dest,dest);
        call updates.insert(src, seq);


      }else{
        dbg(NEIGHBOR_CHANNEL, "Removing Node %d from active list and LinkState. Sequence Reset \n", dest);
        call updates.remove(dest);
      }

      updateRT();

   }


   void updateRT(){
     int i;
     int j;
     int u;
     int v;

     //initialize all costs to max. hops count and if counted to 0 [aka not counted]
     for(v = 0; v < 500; v++){
       rT[v].cost = 500;
       rT[v].hops = 0;
       rT[v].counted = 0;
     }
     //initialize source node
     rT[TOS_NODE_ID].cost = 0;
     //shortest path algorithm
     for(i = 0; i < 499; i++){
       j = minDist();
       rT[j].counted = 1;

       for(u = 0; u < 500; u ++){
          if((rT[u].counted == 0) && (rT[j].cost != 500) && (linkState[j].cost[u] == 1) && ((rT[j].cost + linkState[j].cost[u]) < rT[u].cost)){
            rT[u].cost = rT[j].cost + linkState[j].cost[u];
            if(rT[j].hops != 0){
              rT[u].hops = rT[j].hops;
            }else{
              rT[u].hops = u;
            }
          }
       }
     }

   }

   int minDist(){
       int min = 500;
       int minCount;
       int i;

       for(i = 0; i < 500; i++){
           if((rT[i].counted == 0) && (rT[i].cost <= min)){
               min = rT[i].cost;
               minCount = i;
           }
       }

       return minCount;
   }
   //project 3
   void TCPHandler(pack* myMsg){
     uint8_t i;
     tcp_packet* msg = (tcp_packet*) myMsg->payload;
     tcp_packet tcpPack;
     socket_store_t* socket;
     socket = getSocket(msg->destPort);
     switch(msg->flag){
           case SYN_FLAG:
              dbg(TRANSPORT_CHANNEL,"SYN flag arrived\n");
              if(socket == NULL){
                dbg(TRANSPORT_CHANNEL,"Socket not found\n");
              }
              if(socket->state ==  CLOSED){
                dbg(TRANSPORT_CHANNEL,"Closed Socket recieved SYN\n");
                socket->state = SYN_RCVD;
                socket->dest.addr = myMsg->src;
                socket->dest.port = msg->srcPort;

                tcpPack.srcPort = socket->src.port;
                tcpPack.destPort = msg->srcPort;
                tcpPack.flag = ACK_FLAG;

                makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 18, 4, curSeq, (uint8_t*)&tcpPack, sizeof(tcp_packet));
                call Sender.send(sendPackage, hopCount(myMsg->src));

                socket->prevPack = sendPackage;
                dbg(TRANSPORT_CHANNEL, "ACK sent\n");
              }
            break;


            case ACK_FLAG:
              dbg(TRANSPORT_CHANNEL,"ACK FLAG recieved \n");
              if(socket->state == SYN_SENT){
                dbg(TRANSPORT_CHANNEL,"Setting socket to established\n");
                socket->state = ESTABLISHED;
                socket->bytesSent=0;
                socket->lastAck = myMsg->seq;

                tcpPack.flag = ACK_FLAG;
                tcpPack.destPort = msg->srcPort;
                tcpPack.srcPort = socket->src.port;

                makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 18, 4, curSeq, (uint8_t*)&tcpPack, sizeof(tcp_packet));
                curSeq++;
                call Sender.send(sendPackage, hopCount(myMsg->src));
                socket->prevPack = sendPackage;
                dbg(TRANSPORT_CHANNEL, "ACK sent\n");

              }
              if(socket->state == SYN_RCVD){
                dbg(TRANSPORT_CHANNEL,"Setting socket to LISTEN\n");
                socket->state = LISTEN;
                socket->bytesRcvd = 0;

                tcpPack.flag = ACK_FLAG;
                tcpPack.destPort = msg->srcPort;
                tcpPack.srcPort = socket->src.port;

                makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 18, 4, curSeq, (uint8_t*)&tcpPack, sizeof(tcp_packet));
                curSeq++;
                call Sender.send(sendPackage, hopCount(myMsg->src));
                socket->prevPack = sendPackage;


              }
              if(socket->state == ESTABLISHED){
                makePack(&sendPackage, TOS_NODE_ID, TOS_NODE_ID, 0, 4, 0, "null", PACKET_MAX_PAYLOAD_SIZE);
                for(i = 0; i < WINDOWSIZE; i++){
                  if(socket->outstandPacks[i]==myMsg->seq){
                    dbg(TRANSPORT_CHANNEL,"ACK recieved for packet. remove from queue\n");
                    socket->outstandPacks[i]=0;
                    socket->nonAckPack[i] = sendPackage;
                  }
                }
              }

            break;

         case FIN_FLAG:
          dbg(TRANSPORT_CHANNEL,"FIN: Channed will now close....printing sent data\n");
          for(i = 0; i <socket->lastRcvd; i++){
            printf("%c\n", socket->rcvdBuff[i]);

          }
          dbg(TRANSPORT_CHANNEL,"finished printing\n");
          socket->state = CLOSED;
          break;
          }
  }

   void TCPdata(pack* myMsg){
     uint8_t i, pos, msgByte;
     socket_store_t* socket;
     DATA_PACK* dataPack;
     tcp_packet tcpPack;

     dbg(TRANSPORT_CHANNEL,"TCP Data recieved\n");

     dataPack = (DATA_PACK*)myMsg->payload;
     socket = getSocket(dataPack->port);
     msgByte = (uint8_t)dataPack->size;

     pos = dataPack->arr[0];
     for(i = 0; i <dataPack->size; i++){
       socket->rcvdBuff[pos] = dataPack->arr[i];
       pos++;
       socket->lastRcvd++;
     }

     tcpPack.flag=ACK_FLAG;
     tcpPack.srcPort= socket->src.port;
     tcpPack.destPort=socket->dest.port;

    makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 18, 4, myMsg->seq, (uint8_t*)&tcpPack, sizeof(tcp_packet)); //check sequence here
    call Sender.send(sendPackage, hopCount(myMsg->src));

    socket->prevPack = sendPackage;

   }



   socket_store_t* getSocket(nx_uint16_t port){//find socket with correct src and dest
     socket_store_t* socket;
     uint16_t i;
     uint16_t size = call sockets.getMaxSize();

     for(i = 0; i < size; i++){
       socket = call sockets.getAddress(i);
       if(socket->src.port == port){
         return socket;
       }
     }

   }

   socket_store_t* getSocketAddress(nx_uint16_t addr){   //find listening socket to open connection
     socket_store_t* socket;
     uint16_t i;
     uint16_t size = call sockets.getMaxSize();
     dbg(TRANSPORT_CHANNEL,"Finding Socket Address...\n");

     for(i = 0; i < size; i++){
       socket = call sockets.getAddress(i);
       if(socket->dest.addr == addr){
         return socket;
       }
     }

    }

    uint8_t findClosedSocket(){
      uint16_t i;
      uint16_t size = call sockets.getMaxSize();
      socket_store_t socket;

      for(i = 0; i < size; i++){
        socket = call sockets.get(i);
        if(socket.state == CLOSED){
          return i;
        }
      }

    }

    void closeSockets(){
      uint16_t i;
      uint16_t size = call sockets.getMaxSize();
      socket_store_t socket;
      socket.state = CLOSED;
      socket.bytesRcvd = 0;
      for(i = 0; i < size; i++){
        call sockets.pushfront(socket);
      }

    }

    void ClientHandler(pack* myMsg){
      uint8_t i, pos, msgB;
      socket_store_t* socket;
      tcp_packet tcpPack;
      DATA_PACK* dataPack = (DATA_PACK*)myMsg->payload;

      socket = getSocket(dataPack->port);
      msgB = (uint8_t)dataPack->size;

      dbg(TRANSPORT_CHANNEL,"Handshake on node %d's socket...\n",TOS_NODE_ID);

      for(i = 0; i < msgB; i++){
        socket->cName[socket->numClients][i] = dataPack->arr[i+1];
      }
      socket->cNameSize[socket->numClients] = dataPack->size;
      socket->cPort[socket->numClients] = dataPack->arr[0];
      socket->cNode[socket->numClients] = myMsg->src;

      dbg(TRANSPORT_CHANNEL, "hello %c%c%c %d\r\n",socket->cName[socket->numClients][0],socket->cName[socket->numClients][1],socket->cName[socket->numClients][2], socket->cPort[socket->numClients]);

      socket->numClients++;

    }
    void ClientRequest(pack* myMsg){
      uint8_t i,j,k,os,usrnmSize;
      socket_store_t* socket;
      DATA_PACK* dataPack = (DATA_PACK*)myMsg->payload;
      uint8_t nameSize = dataPack->arr[i];
      uint8_t username[nameSize];
      DATA_PACK dataP;
      bool found;

      socket = getSocket(dataPack->port);
      dbg(TRANSPORT_CHANNEL,"Client Request\n");

      switch ((char)dataPack->arr[0]){
        case 'R':
          dbg(TRANSPORT_CHANNEL,"Clients Requested | Number of clients: %d\n", socket->numClients);
          dataP.port = dataPack->size;
          for(i = 0; i < socket->numClients; i++){
            usrnmSize = socket->cNameSize[i]-1;
            dbg(TRANSPORT_CHANNEL,"Client %d | Name Length %d\n",i, usrnmSize);
            for(j = 0; j < socket->cNameSize[i]; j++){
              dataP.arr[j] = socket->cName[i][j];
            }
            dataP.size = socket->cNameSize[i];
            makePack(&sendPackage, TOS_NODE_ID, myMsg->src, 18, 5, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
            call Sender.send(sendPackage, hopCount(myMsg->src));
            curSeq++;
          }
        break;

        case 'B':
          dbg(TRANSPORT_CHANNEL,"Broadcast message...\n");
          for(i = 0; i < socket->numClients; i++){
            dataP.port = socket->cPort[i];
            dataP.size = dataPack->size;
            for(j = 0; j<dataPack->size; j++){
              dataP.arr[j] = dataPack->arr[j+1];
            }
            makePack(&sendPackage, TOS_NODE_ID, socket->cNode[i], 18, 5, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
            call Sender.send(sendPackage, hopCount(socket->cNode[i]));
            dbg(TRANSPORT_CHANNEL,"Message Sent...\n");
            curSeq++;
          }
        break;

        case 'W':
        dbg(TRANSPORT_CHANNEL,"Whisper message...\n");
        os = 2+dataPack->size;
        for(i = 0; i < socket->numClients; i++){
          dbg(TRANSPORT_CHANNEL,"Client %d | Name Length %d\n",i, socket->cNameSize[i]);
          found = TRUE;
          for(j = 0; j < socket->cNameSize; j++){
            if(dataPack->arr[j+os] != socket->cName[i][j]){
              //dbg(TRANSPORT_CHANNEL,"Not a Match\n");
              found = FALSE;
            }
          }
          if(found == TRUE){
            dbg(TRANSPORT_CHANNEL,"Match found, sending data...\n");
            dataP.port = socket->cPort[i];
            dataP.size = dataPack->size;
            for(j = 0; j < dataPack->size; j++){
              dataP.arr[j] = dataPack->arr[j+2];
            }
            makePack(&sendPackage, TOS_NODE_ID, socket->cNode[i], 18, 5, curSeq, (uint8_t*)&dataP, sizeof(DATA_PACK));
            call Sender.send(sendPackage, hopCount(socket->cNode[i]));
            curSeq++;
          }
        }
        break;
      }

    }


}
