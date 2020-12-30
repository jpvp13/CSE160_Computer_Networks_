#include "../../includes/linkState.h"
#include "../../includes/RoutingTable.h"
configuration routingTableC{

  provides interface routingTable;
}
implementation{

  components new AMReceiverC(AM_PACK) as GeneralReceive;
  components routingTableP as routing;
  routingTable = routing;

  components new HashmapC(uint32_t, 10) as neighborsClone;
  components new HashmapC(uint32_t, 999) as packetsClone;

  routing.neighborsClone -> neighborsClone;
  routing.packetsClone -> packetsClone;

  components new SimpleSendC(AM_PACK);
  Node.Sender -> SimpleSendC;

  components ActiveMessageC;
  Node.AMControl -> ActiveMessageC;


}
