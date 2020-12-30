#include "../../includes/linkState.h"
#include "../../includes/RoutingTable.h"
interface routingTable{
   // Routing Table commands

   //event int updateRoutingTable();
   //event void updateLinkState();
   //event void protocolTwo();

   command void updateNeighborClone(nx_uint16_t node, uint32_t timeStamp);
   command void updatePacketsClone(nx_uint16_t src, nx_uint16_t seq);
   //event void updateLinkStateClone();
   //event void updateLinkUpdateClone();

  // event bool checkLinkState(uint16_t src, uint16_t dest, uint16_t seq, int cost);

   //event int minDist(routingTable rT);
   command bool yes(); //tests for connectivity
}
