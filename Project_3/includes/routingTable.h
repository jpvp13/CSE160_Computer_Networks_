#ifndef ROUTING_TABLE_H
#define ROUTING_TABLE_H

typedef nx_struct routingTable{

  nx_uint16_t cost; //cost of hops
  nx_uint16_t hops;
  nx_uint16_t counted; //"memory" section of table has been checked

} routingTable[500];

#endif
