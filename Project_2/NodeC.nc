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
// #include "includes/neighbors.h"
#include "includes/routingTable.h"


configuration NodeC{
}

implementation {
    components MainC;
    components Node;
    components new AMReceiverC(AM_PACK) as GeneralReceive;



    //Timer Component
    components new TimerMilliC() as ntimer;

   //Project 2 implementations

   components new HashmapC(uint32_t, 10) as neighbors;
   components new HashmapC(uint32_t, 500) as activeNodes;
   components new HashmapC(uint32_t, 500) as packets;
   components new HashmapC(uint32_t, 500) as updates;


    Node -> MainC.Boot;

    //Project 2 implementations
    Node.neighbors -> neighbors;
    Node.activeNodes -> activeNodes;
    Node.packets -> packets;
    Node.updates -> updates;

    Node.Receive -> GeneralReceive;

    components new ListC(int, 500) as cost;
    Node.cost -> cost;

    //#############################

    components ActiveMessageC;
    Node.AMControl -> ActiveMessageC;

    components new SimpleSendC(AM_PACK);
    Node.Sender -> SimpleSendC;

    components CommandHandlerC;
    Node.CommandHandler -> CommandHandlerC;
    
    //Timer instance
    Node.ntimer -> ntimer;
}
