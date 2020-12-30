#include "../../includes/linkState.h"
#include "../../includes/RoutingTable.h"

module routingTableP{

  provides interface routingTable;

  uses interface SimpleSend as Sender;

  uses interface SplitControl as AMControl;
  uses interface Receive;


  uses interface Hashmap<uint32_t> as neighborsClone;
  uses interface Hashmap<uint32_t> as packetsClone;

}

implementation{
/*
  command bool routingTable.yes(){
    return 1;
  }

  command void routingTable.updateNeighborClone(nx_uint16_t node, uint32_t timeStamp){
    if(call neighborsClone.contains(node) == 0){
      call neighborsClone.insert(node, timeStamp);
    }else{
      call neighborsClone.remove(node);
      call neighborsClone.insert(node, timeStamp);
    }
  }

  command void routingTable.updatePacketsClone(nx_uint16_t src, nx_uint16_t seq){
    nx_uint16_t hashSeq;
    if(call packetsClone.contains(src) == 1){ //source has been seen before
      hashSeq = call packetsClone.get(src);
      if(hashSeq > seq){ //packet is a duplicate
        return TRUE;
      }else{ //packet is not duplicate, reset seq
        call packetsClone.remove(src);
        call packetsClone.insert(src, seq);
        return FALSE;
      }
    }else{ //source has not been seen before
      call packetsClone.insert(src,seq);
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

       if((currentTS - nodeTS) > 180000){
         dbg(NEIGHBOR_CHANNEL, "Neighbor %d is offline", neighborKeys[i]);

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

*/
}
