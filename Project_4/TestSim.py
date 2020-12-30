#ANDES Lab - University of California, Merced
#Author: UCM ANDES Lab
#$Author: abeltran2 $
#$LastChangedDate: 2014-08-31 16:06:26 -0700 (Sun, 31 Aug 2014) $
#! /usr/bin/python
import sys
from TOSSIM import *
from CommandMsg import *

class TestSim:
    moteids=[]
    # COMMAND TYPES
    CMD_PING = 0
    CMD_NEIGHBOR_DUMP = 1
    CMD_LINKSTATE_DMP = 2
    CMD_ROUTE_DUMP = 3
    CMD_TEST_CLIENT = 4
    CMD_TEST_SERVER = 5
    CMD_END_CONNECTION = 7
    CMD_CLIENT = 8
    CMD_MESSAGE = 10
    CMD_BROADCAST = 11
    CMD_REQUEST = 12
    CMD_APP_SERVER = 13

    # CHANNELS - see includes/channels.h
    COMMAND_CHANNEL="command";
    GENERAL_CHANNEL="general";

    # Project 1
    NEIGHBOR_CHANNEL="neighbor";
    FLOODING_CHANNEL="flooding";

    # Project 2
    ROUTING_CHANNEL="routing";

    # Project 3
    TRANSPORT_CHANNEL="transport";

    # Personal Debuggin Channels for some of the additional models implemented.
    HASHMAP_CHANNEL="hashmap";

    # Initialize Vars
    numMote=0

    def __init__(self):
        self.t = Tossim([])
        self.r = self.t.radio()

        #Create a Command Packet
        self.msg = CommandMsg()
        self.pkt = self.t.newPacket()
        self.pkt.setType(self.msg.get_amType())

    # Load a topo file and use it.
    def loadTopo(self, topoFile):
        print 'Creating Topo!'
        # Read topology file.
        topoFile = 'topo/'+topoFile
        f = open(topoFile, "r")
        self.numMote = int(f.readline());
        print 'Number of Motes', self.numMote
        for line in f:
            s = line.split()
            if s:
                print " ", s[0], " ", s[1], " ", s[2];
                self.r.add(int(s[0]), int(s[1]), float(s[2]))
                if not int(s[0]) in self.moteids:
                    self.moteids=self.moteids+[int(s[0])]
                if not int(s[1]) in self.moteids:
                    self.moteids=self.moteids+[int(s[1])]

    # Load a noise file and apply it.
    def loadNoise(self, noiseFile):
        if self.numMote == 0:
            print "Create a topo first"
            return;

        # Get and Create a Noise Model
        noiseFile = 'noise/'+noiseFile;
        noise = open(noiseFile, "r")
        for line in noise:
            str1 = line.strip()
            if str1:
                val = int(str1)
            for i in self.moteids:
                self.t.getNode(i).addNoiseTraceReading(val)

        for i in self.moteids:
            print "Creating noise model for ",i;
            self.t.getNode(i).createNoiseModel()

    def bootNode(self, nodeID):
        if self.numMote == 0:
            print "Create a topo first"
            return;
        self.t.getNode(nodeID).bootAtTime(1333*nodeID);

    def bootAll(self):
        i=0;
        for i in self.moteids:
            self.bootNode(i);

    def moteOff(self, nodeID):
        self.t.getNode(nodeID).turnOff();

    def moteOn(self, nodeID):
        self.t.getNode(nodeID).turnOn();

    def run(self, ticks):
        for i in range(ticks):
            self.t.runNextEvent()

    # Rough run time. tickPerSecond does not work.
    def runTime(self, amount):
        self.run(amount*1000)

    # Generic Command
    def sendCMD(self, ID, dest, payloadStr):
        self.msg.set_dest(dest);
        self.msg.set_id(ID);
        self.msg.setString_payload(payloadStr)

        self.pkt.setData(self.msg.data)
        self.pkt.setDestination(dest)
        self.pkt.deliver(dest, self.t.time()+5)

    def ping(self, source, dest, msg):
        self.sendCMD(self.CMD_PING, source, "{0}{1}".format(chr(dest),msg));

    def neighborDMP(self, destination):
        self.sendCMD(self.CMD_NEIGHBOR_DUMP, destination, "neighbor command");

    def routeDMP(self, destination):
        self.sendCMD(self.CMD_ROUTE_DUMP, destination, "routing command");

    def linkStateDMP(self,destination):
        self.sendCMD(self.CMD_LINKSTATE_DMP, destination, "linkstate command");

    def addChannel(self, channelName, out=sys.stdout):
        print 'Adding Channel', channelName;
        self.t.addChannel(channelName, out);

    def testClient(self, source, destination, srcPort, destPort, num):
        self.sendCMD(self.CMD_TEST_CLIENT, source, "{0}{1}{2}{3}{4}".format(chr(destination),chr(srcPort),chr(destPort),chr(num & 0xFF),chr((num>>8) & 0xFF)));

    def testServer(self, source, port):
        self.sendCMD(self.CMD_TEST_SERVER, source, "{0}".format(chr(port)));

    def appServer(self, source, port):
        self.sendCMD(self.CMD_APP_SERVER, source, "{0}".format(chr(port)));

    def endConnect(self, source, dest, srcPort, destPort):
        self.sendCMD(self.CMD_END_CONNECTION, source, "{0}{1}{2}".format(chr(dest),chr(srcPort),chr(destPort)));

    def client(self, source, dest, srcPort, destPort, size, data):
        self.sendCMD(self.CMD_CLIENT, source, "{0}{1}{2}{3}{4}".format(chr(dest),chr(srcPort),chr(destPort),chr(size), data));

    def clientMessage(self, source, srcPort, usrname, size, data):
        self.sendCMD(self.CMD_MESSAGE, source, "{0}{1}{2}{3}".format(chr(srcPort), chr(size), chr(size), data));

    def broadcast(self, source, srcPort, size, data):
        self.sendCMD(self.CMD_BROADCAST, source, "{0}{1}{2}".format(chr(srcPort),chr(size),data));

    def request(self, source, srcPort):
        self.sendCMD(self.CMD_REQUEST, source, "{0}".format(chr(srcPort)));

def main():
    s = TestSim();
    s.runTime(10);
    s.loadTopo("long_line.topo"); #change topology here
    s.loadNoise("no_noise.txt");
    s.bootAll();
    s.addChannel(s.COMMAND_CHANNEL);
    s.addChannel(s.GENERAL_CHANNEL);
    #s.addChannel(s.NEIGHBOR_CHANNEL);
    #s.addChannel(s.FLOODING_CHANNEL);
    s.addChannel(s.ROUTING_CHANNEL);
    s.addChannel(s.TRANSPORT_CHANNEL);

#TCP  [s.setTestClient(self node, destnode, selfport, destnodePort)]


    s.runTime(20);
    #s.testServer(3,1);
    #s.runTime(10);
    #s.testClient(1,3,1,1,16);
    #s.runTime(100);
    #s.endConnect(1,3,1,1);
    #s.runTime(20);
    s.appServer(1, 41);
    s.runTime(30);
    s.client(2, 1, 1, 41, 4, 'ACE');
    s.runTime(20);
    s.runTime(30);
    s.client(3, 1, 1, 41, 4, 'BOB');
    s.runTime(30);
    #s.broadcast(2,1,5,'HELLO');
    #s.runTime(50);
    #s.clientMessage(2,1,3,3,'SUP');
    #s.runTime(50);
    s.request(2,1);
    s.runTime(100);
    #s.routeDMP(2);
    #s.runTime(10);



#how to call event void CommandHandler.printNeighbors(){}
#val in s.neighborDMP(value) determines node being printed
    #s.neighborDMP(4);
    #s.runTime(20);

if __name__ == '__main__':
    main()