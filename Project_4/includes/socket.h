#ifndef __SOCKET_H__
#define __SOCKET_H__

#include "packet.h"

enum{
    MAX_NUM_OF_SOCKETS = 10,
    ROOT_SOCKET_ADDR = 255,
    ROOT_SOCKET_PORT = 255,
    SOCKET_BUFFER_SIZE = 128,
    WINDOWSIZE = 2,
    DATA_SIZE = PACKET_MAX_PAYLOAD_SIZE-2,
};

enum socket_state{
    CLOSED,
    LISTEN,
    ESTABLISHED,
    SYN_SENT,
    SYN_RCVD,
    FIN,
};


typedef nx_uint8_t nx_socket_port_t;
typedef uint8_t socket_port_t;


typedef struct DATA_PACK{
  nx_uint8_t size;
  nx_uint8_t port;
  nx_uint8_t arr[DATA_SIZE];
}DATA_PACK;

// socket_addr_t is a simplified version of an IP connection.
typedef nx_struct socket_addr_t{
    nx_socket_port_t port;
    nx_uint16_t addr;
}socket_addr_t;


// File descripter id. Each id is associated with a socket_store_t
typedef uint8_t socket_t;


// State of a socket.
typedef struct socket_store_t{
    uint8_t flag;
    enum socket_state state;
    //socket_port_t src;
    socket_addr_t src;
    socket_addr_t dest;

    uint8_t numClients;
    uint8_t cName[10][10];
    uint8_t cNameSize[10];
    uint8_t cPort[10];
    uint8_t cNode[10];

    // This is the sender portion.
    uint8_t sendBuff[SOCKET_BUFFER_SIZE];
    uint8_t lastWritten;
    uint8_t lastAck;
    uint8_t lastSent;
    uint8_t numPackets;
    uint8_t seqMsg;
    pack prevPack;

    // This is the receiver portion
    uint8_t rcvdBuff[SOCKET_BUFFER_SIZE];
    uint8_t lastRead;
    uint8_t lastRcvd;
    uint8_t nextExpected;

    uint8_t totalBytes;
    uint8_t bytesSent;
    uint8_t bytesRcvd;

    //track packs
    uint8_t outstandPacks[WINDOWSIZE];
    uint8_t resend[WINDOWSIZE];
    pack nonAckPack[WINDOWSIZE];

    uint16_t RTT;
    uint8_t effectiveWindow;
    uint16_t timeout;
    uint16_t resets;
}socket_store_t;

#endif
