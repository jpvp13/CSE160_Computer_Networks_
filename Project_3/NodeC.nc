/**
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */

#include <Timer.h>
#include "includes/CommandMsg.h"
#include "includes/packet.h"
#include "includes/socket.h"

configuration NodeC{ //NodeC stores "connections" to node components
}
implementation {
    components MainC;
    components Node;
    components new AMReceiverC(AM_PACK) as GeneralReceive;

    components new HashmapC(uint32_t, 10) as neighbors;
    components new HashmapC(uint32_t, 500) as activeNodes;
    components new HashmapC(uint32_t, 500) as packets;
    components new HashmapC(uint32_t, 500) as updates;
    Node.neighbors -> neighbors;
    Node.activeNodes -> activeNodes;
    Node.packets -> packets;
    Node.updates -> updates;

    components new ListC(int, 500) as cost;
    Node.cost -> cost;
    components new ListC(socket_store_t, MAX_NUM_OF_SOCKETS) as socketList;
    Node.sockets -> socketList;

    //Timer component
    components new TimerMilliC() as ntimerC;
    components new TimerMilliC() as tTimer;


    Node -> MainC.Boot;

    Node.Receive -> GeneralReceive;
    //timer instance initiation for each node
    Node.ntimer -> ntimerC;
    Node.tTimer -> tTimer;


    components ActiveMessageC;
    Node.AMControl -> ActiveMessageC;

    components new SimpleSendC(AM_PACK);
    Node.Sender -> SimpleSendC;

    components CommandHandlerC;
    Node.CommandHandler -> CommandHandlerC;

    //components routingTableC as routing;
    //Node.routing -> routing;



}
