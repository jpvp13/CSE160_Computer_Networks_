/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */


 /*#################################
            Main Code
 ###################################*/

#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
//#include "neighbor.h"   //created to use with flooding/neighbor discover


            //creates nodes that use the filename and can rename files to something simpiler.
module Node{ //each node uses these interferces add interfaces with <uses inteferace>
   uses interface Boot;

   uses interface SplitControl as AMControl;

   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface Timer<TMilli> as ntimer; //gets each node to use the timer instance

   uses interface CommandHandler;

   //
   // //test files
   // //this line allows code in node to use the commands provided by the sampleModule
   // uses interface sampleModule as sampleMod;
   //

}

implementation{   //code that each node runs | implementation of node module
   pack sendPackage;

   //Project 1 things
   uint16_t recievedSeq[100]; //this can be considered "cache" for each node to remember if it hsa recieved the package in question
   uint16_t recievedSrc[100]; //this can be considered "cache" for each node to remember if it hsa recieved the package in question

   int neighbors[15];   //this is a array that holds all info of each node
   int neighborCount;  //this is keeping track of how many nodes are discovered via flooding
   int recievedCount = 0;  //this keeps track of the protocols

   bool checkPackage(uint16_t src, uint16_t seq);
   void addNeighbor(uint16_t node);

   

   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);

   event void Boot.booted(){

      //bool testSampleModule = 0;

      call AMControl.start();
      call ntimer.startPeriodic(6000);


      dbg(GENERAL_CHANNEL, "Booted\n");

      //testSampleModule = call sampleMod.sampleFunction();
      //dbg(GENERAL_CHANNEL, "Sample module connected = %d \n", testSampleModule); 

   }

   event void AMControl.startDone(error_t err){    //once called node starts "listening"
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   // //###################
   // //timer
   // event void ntimer.fired(){ //starts with ntimer.startPeriodic | run neighbor discovery: send packets and decement current discovery value
   //    dbg(GENERAL_CHANNEL, "Timer Fired on node = %d \n", TOS_NODE_ID);
   // }


   event void AMControl.stopDone(error_t err){}


   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){ //this code will onyl run when the event is triggered

      //IMPORTANT DEBUGGING
     // dbg(GENERAL_CHANNEL, "Packet Received\n");

      if(len == sizeof(pack)){
         pack* myMsg=(pack*) payload;

         //dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);

         //this checks if message output from source destination is anything but 0
         if(myMsg->dest != 0){
            dbg(FLOODING_CHANNEL, "Package sent from: %d\n" , myMsg->src); //this prints from which source node a package is being sent from
            dbg(FLOODING_CHANNEL, "Package Sequence number: %d\n", myMsg->seq);  //this prints the sequence # of said package
            dbg(FLOODING_CHANNEL, "Package TTL: %d\n", myMsg->TTL);  //this prints the TTL of a package

            if(myMsg->dest == TOS_NODE_ID){   //this checks if once a node recieves a package if its at correct destination then its responds "REACHED!"
               dbg(FLOODING_CHANNEL, "Package destination has been reached!\n");

               return msg;

               //this checks if TTl is larger than 0 and if while checking package info if it is either the source and sequence value as well as if the node is the source 
               //if true, then flooding needs to continue to discover all neighbors
               //Flooding is done 
            } else if(myMsg->TTL > 0 && !(checkPackage(myMsg->src, myMsg->seq)) && (myMsg->src != TOS_NODE_ID)){   
               dbg(FLOODING_CHANNEL, "Currently Flooding....\n"); //begins sending out packages to other neighbors

               makePack(&sendPackage, myMsg->src, myMsg->dest, (myMsg->TTL - 1), 0, myMsg->seq, myMsg->payload, PACKET_MAX_PAYLOAD_SIZE);

               call Sender.send(sendPackage, AM_BROADCAST_ADDR);

               return msg;

            } else {
               dbg(FLOODING_CHANNEL, "TTL means termination / This is a duplicate package!\n");
               return msg;
            }



         }
         //dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);

         //#####################
         //neighbor disovery
         //#####################
         //dbg(NEIGHBOR_CHANNEL, "Neighbor is: %d\n", myMsg->src);

         //IMPORTANT DEBUGGING

         //printf("I am source: %d\n", myMsg->src);
         //dbg(NEIGHBOR_CHANNEL, "I am node: %d\n", myMsg->src);

         addNeighbor(myMsg->src);   //call addNeighbor function

         //IMPORTANT DEBUGGING
         //dbg(NEIGHBOR_CHANNEL, "Number of neighbors surrounding me: %d\n", neighborCount);

         return msg;
      }
      dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
      return msg;
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 19, 0, (sendPackage.seq + 1), payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, AM_BROADCAST_ADDR);
   }

   //important part of being able to send packets
   event void ntimer.fired(){ //starts with ntimer.startPeriodic | run neighbor discovery: send packets and decement current discovery value

     makePack(&sendPackage, TOS_NODE_ID, 0, 0, 0, 0, "Timer", PACKET_MAX_PAYLOAD_SIZE);

     neighborCount = 0; //this is 

     //dbg(GENERAL_CHANNEL, "Timer Fired on node = %d \n", TOS_NODE_ID);
     call Sender.send(sendPackage, AM_BROADCAST_ADDR);
   }

   event void CommandHandler.printNeighbors(){
      int i;
      printf("###########PRINTING NEIGHBORS#################\n");
      printf("The neighbors of node: %d\n", TOS_NODE_ID);
      
      for(i = 0; i < 15; i++){
         if(neighbors[i] > 0){
            printf("Neighbor (%d) is: %d\n", i, neighbors[i]);
           // printf("HELPPPP ;/\n");

         } 
         else {
            printf("There are no more neighbors :( \n");     //this just visual prints out that there are no neighbors
         }
      
      }
      printf("\n");

   }

   event void CommandHandler.printRouteTable(){}

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}


   //this physically creates a packet to be sent throughout the nodes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }

   //this is used to add all neighbors to our known list
   void addNeighbor(uint16_t node){
      int i;
      int found = 0;

      for(i = 0; i < 15; i++){
         if(node == neighbors[i]){   //checks to see if the current node is already inside our known nodes
            found = found + 1;         //if so, it increases found for "discovered" nodes
         }
      }

      if(found == 0){   
         neighbors[neighborCount] = node;    //if no nodes are discovered, then found stays 0
         neighborCount = neighborCount + 1;     //thus this initializes node to whichever node in our table existsi (or doesnt)
      }
   }

   //checks to make sure packet recieved is valid
   // checks if duplicate or not
   bool checkPackage(uint16_t src, uint16_t seq){
     int i;
   	for(i = 0; i < 100; i++){
         if(src == recievedSrc[i] && seq == recievedSeq[i]){
            return TRUE;   //this means that the node has received a specific package and stops it from moving forward
         }
      }

      recievedSrc[recievedCount] = src;   //this sets the src value to what the current counter is to sync everything together
   	recievedSeq[recievedCount] = seq;   //this sets the seq value to what the cirrent counter is to sync eberything together
 

      if(recievedCount != 99){   //this checks to make sure we are within the set of protocols defined
         recievedCount = recievedCount + 1;  //moves the counter by 1 to move onto next protocol
      } else {
         recievedCount = 0; //resets counter back to 0 to begin protocols again
      }

      return FALSE; //this means that the node has NOT receieved a simialar packet, and allows it to move forward
   
   }
}


