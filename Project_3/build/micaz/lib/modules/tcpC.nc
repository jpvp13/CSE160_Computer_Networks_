#include<Timer.h>
#include "includes/socket.h"
#include "includes/CommandMsg"
#include "includes/packetTCP.h"

configuration tcpC{

  provides interface tcp;
}
implementation{

  components tcpP as Tcp;
  tcp = Tcp;

  components new SimpleSendC(AM_PACK);
  Tcp.Sender -> SimpleSendC;

  components new TimerMilliC() as ntimerC;
  Tcp.ntimer -> ntimerC;

  components new AMReceiverC(AM_PACK) as GeneralReceive;
  Tcp.Receive -> GeneralReceive;

  components CommandHandlerC;
  Tcp.CommandHandler -> CommandHandlerC;

  components new ListC(socket_store_t) as Sockets;
  Tcp.Sockets -> Sockets;
}
