#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/socket.h"
#include "includes/packetTCP.h"
#include "includes/sendInfo.h"
#include "includes/routingTable.h"

module tcpP{

  provides interface tcp;
  uses interface SimpleSend as Sender;
  uses interface CommandHandler;
  uses interface Receive;
  uses interface List<socket_store_t> as Sockets;
  uses interface Timer<TMilli> as ntimer;

}

implementation{

  routingTable myRoutingTable;

  void connect(socket_store_t mySocket); //connect to server
  void finishConnect(socket_store_t mySocket); //connected + send data
  socket_store_t getSocket(nx_uint16_t srcS, nx_uint16_t destS); //find socket with correct src and dest
  socket_store_t serverSocket(nx_uint16_t destS); //find listening socket to open connection
  bool searchRt(routingTable * rT, nx_uint16_t dest); //search for destination

  command void tcp.tcpHandler(pack * myMsg){
    tcp_packet * msg = (tcp_packet *) myMsg -> payload;
    socket_store_t mySocket;
    uint16_t srcPort;
    uint16_t destPort;
    uint16_t seq;
    uint16_t numAck;
    uint16_t flag;

    seq = myMsg -> seq;
    srcPort = myMsg -> srcPort;
    destPort = myMsg -> destPort;
    numAck = myMsg -> ACK;
    flag = myMsg -> flag;

    //3 way handshake
    if(flag == SYN_FLAG || flag == ACK_FLAG || flag == SYN_ACK_FLAG){

        switch(flag){

          case SYN_FLAG:
            break;

          case ACK_FLAG:
            break;

          case SYN_ACK_FLAG:
            break;
        }

    }

    //Transmission + ACKs
    if(flag == DATA_FLAG || flag = DATA_ACK_FLAG){

      switch(flag){

        case DATA_FLAG:
          break;

        case DATA_ACK_FLAG:
          break;

        default:
          break;
      }
    }

    //Connection Teardown
    if(flag == FIN_FLAG || flag == ACK_FIN_FLAG){

      switch(flag){

        case FIN_FLAG:
          break;

        case ACK_FIN_FLAG:
          break;
      }
    }
  }
  command void tcp.getRt(routingTable rT){
    myRoutingTable = rT;
  }

  command void tcp.printTestClient(){

  }
  command void tcp.printTestServer(){

  }

  void connect(socket_store_t mySocket){

  }
  void finishConnect(socket_store_t mySocket){ //connected. send data here

  }
  socket_store_t getSocket(nx_uint16_t srcS, nx_uint16_t destS){

  }
  socket_store_t serverSocket(nx_uint16_t destS){

  }
  bool searchRT(routingTable *rT, int dest){//does my destination exist?
      if(rT[dest].hops < 500){
        return true;
      }else{
        return false;
      }
    }

  }
