#include "../../packet.h"
#include "../../routingTable.h"

interface tcp{
  command void tcpHandler(pack *myMsg);
  command void getRt(routingTable rT);
  command void printTestServer();
  command void printTestClient();
}
