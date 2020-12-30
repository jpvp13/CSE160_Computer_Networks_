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
#include "includes/linkState.h"   //Project 2
#include "includes/routingTable.h"  //Project 2

module Node{ //each node uses these interfaces add interfaces with <uses interface>
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;

   uses interface Hashmap<uint32_t> as neighbors; //iterfaces to use hashmap
   uses interface Hashmap<uint32_t> as activeNodes; //iterfaces to use hashmap
   uses interface Hashmap<uint32_t> as packets; //iterfaces to use hashmap
   uses interface Hashmap<uint32_t> as updates; //iterfaces to use hashmap

   uses interface List<int> as cost;  //the cost of 

   uses interface Timer<TMilli> as ntimer; //gets each node to use the timer instance


}

implementation{ //code that each node runs | implementation of node module
   pack sendPackage;
   routingTable rT;
   linkState linkState = {0};
   int linkCost;


   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);

   //flood + neighbor discovery | Project 1
   bool checkPackage(nx_uint16_t src, nx_uint16_t seq);
   void addNeighbor(nx_uint16_t node);

   //routingTable | Project 2 
   void updateRT();
   void protocoltwo(); //broadcast neighbors to other nodes
   void updateLS(uint16_t src, uint16_t dest, uint16_t seq, int dist);  //Update LinkState
   bool checkLS(uint16_t src, uint16_t dest, uint16_t seq, int dist); //checks LinkState
   void checkTS(); //check TimeStamp , see if node is alive
   int hopCount(int dest);  //finds cost of counts
   int minDist();   //finds min distance between nodes


   event void Boot.booted(){ //each node starts when boot event is activated [activated by python end]

      call AMControl.start(); //calls AMControl | be called <call>
      call ntimer.startPeriodic(60000); //time in miliseconds

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
          /*dbg(FLOODING_CHANNEL, "Package Sent. Sending from: %d to %d\n", myMsg->src, myMsg->dest);
          dbg(FLOODING_CHANNEL, "TTL: %d\n", myMsg->TTL);*/

          //check if package is at destination  
          if((myMsg->dest == TOS_NODE_ID) && !(checkPackage(myMsg ->src, myMsg ->seq))){  //checks if destination has reached intended node, and if package is not recieved
            dbg(ROUTING_CHANNEL, "Packet destination reached. Payload: %s \n", myMsg->payload);
            if(myMsg ->protocol == 0){  //protocol 2 is routing

              //dbg(ROUTING_CHANNEL, "Reply Ping\n");

              makePack(&sendPackage, TOS_NODE_ID, myMsg ->src, 18,1,(sendPackage.seq+1),"package here",PACKET_MAX_PAYLOAD_SIZE);  //sends the package through
              call Sender.send(sendPackage, hopCount(myMsg->src));  //finds the hopcount
              return msg;
            }
            return msg;
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
     uint32_t timeStamp = call ntimer.getNow(); //makes TimeStamp
     makePack(&sendPackage, TOS_NODE_ID, 0, 0, 0, (sendPackage.seq + 1), "Neighbor", PACKET_MAX_PAYLOAD_SIZE);  
     call Sender.send(sendPackage, AM_BROADCAST_ADDR);

     if(timeStamp > 180000){
       protocoltwo(); //boradcast neighbors
     }
     checkTS(); //checks TimeStamp
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 18, 0, (sendPackage.seq+1), payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, hopCount(destination)); //pings the hopCount
   }

   event void CommandHandler.printNeighbors(){
     int i = 0;
     uint16_t j= call neighbors.size();
     uint32_t* node = call neighbors.getKeys();
     printf("############ PRINTING NEIGHBORS FOR NODE %d ############\n", TOS_NODE_ID);

     for(i; i < j; i++){
       printf("Neighbor (%d) is: %d\n", i, node[i]);
     }
   }

   event void CommandHandler.printRouteTable(){
     int i;
     printf("############ ROUTING TABLE FOR NODE %d ###############\n", TOS_NODE_ID);
     printf("Node\tCost\tHop\n"); //makes the table nice

     for(i = 0; i < 500 ; i++){
       if(rT[i].cost < 500){
        printf("---%d\t---%d\t---%d\n", i, rT[i].cost, rT[i].hops); //makes the table nice
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
     printf("########## Link State ##########\n");

     for(i = 0; i < counter; i++){
       currentNode = currentNodes[i]; //sets which node currently on

       for(j = 0; j < counter; j++){

         neighborNodes = currentNodes[j]; //sets the current node we are on as the neighborNode for the previous node

         if(linkState[currentNode].cost[neighborNodes] != 0){ //checks that the node is not dead, ommitted from the topo
           printf("Node %d and Node %d are connected \n", currentNode, neighborNodes);
         }
       }
     }

   }

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}




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

     if(call neighbors.contains(node) == 0){  //if neighbor doesn't contain node then insert the new node and its TimeStamp
       call neighbors.insert(node, timeStamp);

     }else{

       call neighbors.remove(node); //if it does contain the node in the neighbor, then remove it and insert the neighbors node and TimeStamp
       call neighbors.insert(node, timeStamp);
     }

     //node is back online
     if(call neighbors.contains(node) == 0 && timeStamp > 60000*5){
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

   int hopCount(int dest){  //returns the countHop
     return rT[dest].hops;
   }


   void protocoltwo(){
     int* neighborNode = call neighbors.getKeys();
     int counter = call neighbors.size();
     int i = 0;

     for(i; i < counter; i++){
       makePack(&sendPackage, TOS_NODE_ID, neighborNode[i], 18, 2, (sendPackage.seq + 1), "1", PACKET_MAX_PAYLOAD_SIZE); //node is online
       call Sender.send(sendPackage, AM_BROADCAST_ADDR);  //broadcast the package into node and its neighbor
     }
     checkTS(); //check the TimeStamp
   }

   void checkTS(){    //checks TimeStamp 
     int i;
     int neighborIndex = call neighbors.size();
     int* neighborKeys = call neighbors.getKeys();
     uint32_t currentTS = call ntimer.getNow();
     uint32_t nodeTS;

     for(i = 0; i < neighborIndex; i++){//remove neighbor if not online
       nodeTS = call neighbors.get(neighborKeys[i]);

       if((currentTS - nodeTS) > 180000){
         dbg(NEIGHBOR_CHANNEL, "Neighbor %d is offline", neighborKeys[i]);

         //message about offline neighbor
         makePack(&sendPackage, TOS_NODE_ID,neighborKeys[i], 18, 2, (sendPackage.seq+1), "0", PACKET_MAX_PAYLOAD_SIZE);
         call Sender.send(sendPackage, AM_BROADCAST_ADDR);

         updateLS(TOS_NODE_ID, neighborKeys[i], (sendPackage.seq + 1), "0");
       }//neighbor is online, do nothing
     }
   }

   bool checkLS(uint16_t src, uint16_t dest, uint16_t seq, int dist){ //checks to make sure that node has a connection to another node
     if(call updates.contains(src) == 1){
       if(call updates.get(src) < seq){
         return TRUE;
       }else{
         return FALSE;
       }
     }

     return TRUE;
   }

   void updateLS(uint16_t src, uint16_t dest, uint16_t seq, int dist){  //Update the connection between nodes and neighbors
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

   int minDist(){   //return the minDistance of a hop
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

}
