#ifndef PACKETTCP_H
#define PACKETTCP_H

#include "channels.h"
#include "protocol.h"

enum flag_state{
DATA_FLAG = 0,
SYN_FLAG = 1,
SYN_ACK_FLAG = 2,
ACK_FLAG = 3,
FIN_FLAG = 4,
ACK_FIN_FLAG = 5,
DATA_ACK_FLAG = 6,
};

typedef nx_struct tcp_packet{

  nx_uint8_t advertisedWindow;
  nx_uint8_t seq;
  nx_uint8_t destPort;
  nx_uint8_t srcPort;
  nx_uint8_t flag;
  nx_uint8_t ack;
  nx_uint16_t payload[6];

}tcp_packet;

#endif
