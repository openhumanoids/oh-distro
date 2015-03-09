#!/bin/env python

#**********************************************************************
# $Id: dce_ctl.py 50 2015-02-18 08:18:11Z karl $
#**********************************************************************

# If the servercomm module isn't in the same directory as this module
# then we need to add its path location to Python's module search path using
# the following lines:

#import sys
#sys.path.insert(1, "/usr/local/lib/iwl/nstdigui")

# The servercomm module contains many helper methods and functions to
# control the impairment engine.

#**********************************************************************
# setimpair Flow Interface [zeroztats] Impairment
#
# Impairment := "forwardall" | "dropall" | [Drop] [Dup] [Delay] [Rate] [Jitter] [Alter] [Corrupt]
#
# Drop := "drop" percent ["couple" percent CouplingInterval ["slew"]]
#
# Dup := "dup" percent ["couple" percent]
#
# Delay := "delay" uint32
#
# Rate := "rate" XmitRate MinPayload MaxPayload Overhead MaxQLength ActualRate ActualDuration
#
# The rate parameters control the transmission so that, on
# average, no more than XmitRate bits/second are transmitted
# out the opposite interface (a rate of zero disables rate
# impairment). When an Ethernet frame is received, only the
# bits in its data section are included in the rate computation -
# the frame preamble, header, FCS, and so on, are ignored. To
# accurately compute the packet delays, the number of
# overhead bits in the emulated outgoing link layer need to be
# included in the rate computation. To calculate the number of
# such overhead bits, the following aspects of the emulated link
# layer must be provided: the minimum and maximum payload
# bit size (MinPayload and MaxPayload, respectively) and the
# average number of bits in the emulated link layer's headers
# and and tails, if any (in Overhead.) To emulate discards
# properly, the maximum emulated transmission queue bit
# length must be provided in QLength. Lastly, for highest fidelity
# the actual measured transmission rate between Maxwell and
# the final endpoint is entered as ActualRate bits/second and
# actual measured transit delay is entered as ActualDuration
# microseconds. Note that setting ActualRate to zero excludes
# both ActualRate and ActualDuration from the rate
# computations.
#
# Jitter := "jitter" percent JitterTime ["reorderok"] ["couple" percent CouplingInterval ["slew"]]
#**********************************************************************

import time

import servercomm

#DEBUGGING=True
DEBUGGING=False

#**********************************************************************
# DCE
#**********************************************************************
class DCE(object):
    
    NumDummyFlows = 5

    def __init__(self, dce_address):
        # IP address of the Maxwell Pro that is acting as the DCE.
        self.__DCEAddress = dce_address
        self.__DummyMode = False if self.DCEAddress else True
        self.__SrvrCtrl = None

    @property
    def DCEAddress(self):
        return self.__DCEAddress

    @property
    def IsDummyMode(self):
        return self.__DummyMode

    @property
    def SrvrCtrl(self):
        return self.__SrvrCtrl

    @property
    def IsConnected(self):
        return self.__SrvrCtrl is not None

    def Connect(self):
        if self.IsDummyMode:
            return
        if self.__SrvrCtrl is None:
            # Now we create control objects that connect to the server engines and set the
            # standard impairment mode for both directions.
            # A ServerControl instance is needed for each Maxwell Pro we wish to control.
            # The constructor takes the IP address of the Maxwell Pro as an argument.
            self.__SrvrCtrl = servercomm.ServerControl(self.DCEAddress)
            self.__SrvrCtrl.Reconnect()

            # We need at least 5 flows (10 slots).
            startArgs = self.__SrvrCtrl.GetStartArgs()
            if startArgs.itemCount < 10:
                startArgs.itemCount = 10
                self.__SrvrCtrl.RestartServer(startArgs)

    def Disconnect(self):
        if self.IsDummyMode:
            return
        if self.SrvrCtrl is not None:
            # When we exit this script the impairment engine will continue to apply the
            # last impairments we ordered. An explicit disconnect is not needed for this
            # simple example since that will happen when the script ends, but we include
            # it here for demonstration purposes:
            self.__SrvrCtrl.ServerDisconnect()
            self.__SrvrCtrl = None

    def GetNumFlows(self):
        if self.IsDummyMode:
            return self.__class__.NumDummyFlows
        assert self.IsConnected
        return self.SrvrCtrl.GetNumFlows()

    def GetAllStats(self):
        if self.IsDummyMode:
            return None # **TEMPORARY**
        assert self.IsConnected
        return self.SrvrCtrl.GetAllStats()

    def ClearAllFlows(self):
        if self.IsDummyMode:
            return
        assert self.IsConnected
        # Generally it is a good idea to reset Maxwell to a known state before
        # setting up a specific flow impairment. We perform the reset using 
        # the command ClearAllFlows.
        self.SrvrCtrl.ClearAllFlows()

#**********************************************************************
# DCEControl
#**********************************************************************
class DCEControl(DCE):

    # Interface assignments:
    RobotToTeam = 0
    TeamToRobot = 1

    # Flow number assignments:
    FastPath = 0
    SlowPath = 1
    IcmpBypass = 2
    ArpBypass = 3
    BitBucket = 4

    # Ports 16384 - 24575
    FastpathPortval = "0x4000"
    FastpathPortmask = "0xE000"

    # Ports 0 - 2047
    SlowpathPortval = "0x0000"
    SlowpathPortmask = "0xF800"

    def __init__(self, dce_address,
                 fast_queue_length=16*1500*8,
                 # Typical router queue may be 16 packets of 1500 bytes.
                 slow_queue_length=5000*8,
                 # Actual negotiated link rate of data ports. Assume 1 Gig.
                 phys_rate=1000000000,
                 # Minimum payload data size for Ethernet.
                 min_payload=60*8,
                 # Maximum payload data size for Ethernet.
                 max_payload=1500*8,
                 # Ethernet header and tail overhead bits:
                 # preamble(8) + dst(6) + src(6) + length(2) + CRC(4) + interframe gap(12)
                 # overhead=(8 + 6 + 6 + 2 + 4 + 12)*8
                 # dst(6) + src(6) + length(2)
                 overhead=(6 + 6 + 2)*8
                ):

        super(DCEControl, self).__init__(dce_address)

        # Rate limit queue sizes, in bytes
        self.FastQLen = fast_queue_length
        self.SlowQLen = slow_queue_length

        self.PhysRate = phys_rate
        self.MinPayload = min_payload
        self.MaxPayload = max_payload
        self.Overhead = overhead

    #******************************
    # Fast path (Flow 0)
    #******************************
    def InitFastPath(self):
        # Initial is:
        #  100% drop for team==>robot
        #  No impairments for robot==>team

        if self.IsDummyMode:
            return

        assert self.IsConnected

        # Team==>Robot:
        # Never opened for traffic at any time
        # We set the match to "none" so that any packets fall through
        # to the next flow (i.e. to the Slow Path.)
        self.SrvrCtrl.SetMatch(self.__class__.FastPath,
                               self.__class__.TeamToRobot,
                               "none",
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.TeamToRobot,
                                "drop 80" if DEBUGGING else "drop 100",
                                zeroStats = True)
        self.SrvrCtrl.ClearQueue(self.__class__.FastPath,
                                 self.__class__.TeamToRobot)

        # Robot==>Team:
        self.SrvrCtrl.SetMatch(self.__class__.FastPath,
                               self.__class__.RobotToTeam,
                               "ipv4 port dst %s %s" % (self.__class__.FastpathPortval,
                                                        self.__class__.FastpathPortmask),
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.RobotToTeam,
                                "rate 300000000 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.FastQLen, self.PhysRate),
                                zeroStats = True)

    def EstablishFastPathOutsideOpen(self):
        if self.IsDummyMode:
            return

        # Robot==>Team:
        # Set 300,000,000 bits/second rate
        assert self.IsConnected
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.RobotToTeam,
                                "rate 300000000 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.FastQLen, self.PhysRate),
                                zeroStats = False)
        # Team==>Robot:
        # We drop everything in the reverse direction
        # The SetMatch for this direction should be "none", so there should be no
        # packets flowing here.
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.TeamToRobot,
                                "drop 82" if DEBUGGING else "drop 100",
                                zeroStats = False)

    def EstablishFastPathOutsideBlackout(self):
        if self.IsDummyMode:
            return

        # Same as Fast Path Open, Outside
        self.EstablishFastPathOutsideOpen()

    def EstablishFastPathInsideOpen(self):
        if self.IsDummyMode:
            return

        # Robot==>Team:
        # Set 300,000,000 bits/second rate
        assert self.IsConnected
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.RobotToTeam,
                                "rate 300000000 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.FastQLen, self.PhysRate),
                                zeroStats = False)
        # Team==>Robot:
        # We drop everything in the reverse direction
        # The SetMatch for this direction should be "none", so there should be no
        # packets flowing here.
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.TeamToRobot,
                                "drop 84" if DEBUGGING else "drop 100",
                                zeroStats = False)


    def EstablishFastPathInsideBlackout(self):
        if self.IsDummyMode:
            return

        # Robot==>Team:
        # Set 100% drop, clear queue
        assert self.IsConnected
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.RobotToTeam,
                                "drop 86" if DEBUGGING else "drop 100",
                                zeroStats = False)
        self.SrvrCtrl.ClearQueue(self.__class__.FastPath,
                                 self.__class__.RobotToTeam)
        # Team==>Robot:
        # We drop everything in the reverse direction
        self.SrvrCtrl.SetImpair(self.__class__.FastPath,
                                self.__class__.TeamToRobot,
                                "drop 88" if DEBUGGING else "drop 100",
                                zeroStats = False)

    #******************************
    # Slow path (Flow 1)
    #******************************
    def InitSlowPath(self):
        if self.IsDummyMode:
            return

        assert self.IsConnected
        slowpath_portmask = "0xF800"

        # Team==>Robot:
        self.SrvrCtrl.SetMatch(self.__class__.SlowPath,
                               self.__class__.TeamToRobot,
                               "ipv4 port both %s %s" % (self.__class__.SlowpathPortval,
                                                         self.__class__.SlowpathPortmask),
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.SlowPath,
                                self.__class__.TeamToRobot,
                                # OLD: "delay 1000000 rate 9600 %d %d %d %d %d 0" %
                                "rate 9600 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.SlowQLen, self.PhysRate),
                                zeroStats = True)

        # Robot==>Team:
        self.SrvrCtrl.SetMatch(self.__class__.SlowPath,
                               self.__class__.RobotToTeam,
                               "ipv4 port both %s %s" % (self.__class__.SlowpathPortval,
                                                         self.__class__.SlowpathPortmask),
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.SlowPath,
                                self.__class__.RobotToTeam,
                                "rate 9600 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.SlowQLen, self.PhysRate),
                                zeroStats = True)

    #******************************
    # ICMP bypass
    #******************************
    def InitIcmpBypass(self):
        if self.IsDummyMode:
            return

        assert self.IsConnected
        self.SrvrCtrl.SetMatch(self.__class__.IcmpBypass,
                               self.__class__.TeamToRobot,
                               "ipv4 proto 1 0xFF",
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.IcmpBypass,
                                self.__class__.TeamToRobot,
                                # OLD: "delay 1000000 rate 4800 %d %d %d %d %d 0" %
                                "rate 4800 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.SlowQLen, self.PhysRate),
                                zeroStats = True)

        self.SrvrCtrl.SetMatch(self.__class__.IcmpBypass,
                               self.__class__.RobotToTeam,
                               "ipv4 proto 1 0xFF",
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.IcmpBypass,
                                self.__class__.RobotToTeam,
                                "rate 4800 %d %d %d %d %d 0" %
                                (self.MinPayload, self.MaxPayload, self.Overhead,
                                 self.SlowQLen, self.PhysRate),
                                zeroStats = True)

    #******************************
    # ARP bypass
    #******************************
    def InitArpBypass(self):
        if self.IsDummyMode:
            return

        assert self.IsConnected
        self.SrvrCtrl.SetMatch(self.__class__.ArpBypass,
                               self.__class__.TeamToRobot,
                               "lan type 0x0806 0xFFFF",
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.ArpBypass,
                                self.__class__.TeamToRobot,
                                "forwardall",
                                zeroStats = True)

        self.SrvrCtrl.SetMatch(self.__class__.ArpBypass,
                               self.__class__.RobotToTeam,
                               "lan type 0x0806 0xFFFF",
                               zeroStats = False)
        self.SrvrCtrl.SetImpair(self.__class__.ArpBypass,
                                self.__class__.RobotToTeam,
                                "forwardall",
                                zeroStats = True)

    #******************************
    # Bit Bucket /dev/null
    #******************************
    def InitBitBucket(self):
        if self.IsDummyMode:
            return

        assert self.IsConnected
        self.SrvrCtrl.SetMatch(self.__class__.BitBucket,
                               self.__class__.TeamToRobot,
                               "all", 0)
        self.SrvrCtrl.SetImpair(self.__class__.BitBucket,
                                self.__class__.TeamToRobot,
                                "drop 90" if DEBUGGING else "drop 100",
                                zeroStats = True)

        self.SrvrCtrl.SetMatch(self.__class__.BitBucket,
                               self.__class__.RobotToTeam,
                               "all", 0)
        self.SrvrCtrl.SetImpair(self.__class__.BitBucket,
                                self.__class__.RobotToTeam,
                                "drop 92" if DEBUGGING else "drop 100",
                                zeroStats = True)

###############################################################################

#**********************************************************************
# Entry
#**********************************************************************
if __name__ == "__main__":

    import argparse
    import traceback

    # Initialize all the flows:

    parser = argparse.ArgumentParser(description="DCE test.",
                                   usage="%(prog)s [options]  Use -h for help.")

    parser.add_argument("maxpro_hostname",  metavar="<Maxwell Pro host name or IP address>",
			help='The host name or IP address of the Maxwell Pro.')

    pargs = parser.parse_args()

    dce_ctrl = DCEControl(pargs.maxpro_hostname)

    dce_ctrl.Connect()

    print "Dummy", dce_ctrl.IsDummyMode
    astats = dce_ctrl.GetAllStats()
    print type(astats)
    print "ASTATS", astats.stats
    print "ITFSTATS", astats.itfStats
    dce_ctrl.Disconnect()
    import sys
    sys.exit(0)

    dce_ctrl.InitFastPath()	# Flow 0
    dce_ctrl.InitSlowPath()	# Flow 1
    dce_ctrl.InitIcmpBypass()	# Flow 2
    dce_ctrl.InitArpBypass()	# Flow 3
    dce_ctrl.InitBitBucket()	# Flow 4

    # Some made-up intermittent stuff on the fast path:
    dce_ctrl.EstablishFastPathOutsideOpen()
    time.sleep(5)
    dce_ctrl.EstablishFastPathOutsideBlackout()
    time.sleep(5)

    try:
        while True:
            #--#dce_ctrl.OpenFastPath()
            dce_ctrl.EstablishFastPathInsideOpen()
            time.sleep(2)
            #--#dce_ctrl.CloseFastPath()
            dce_ctrl.EstablishFastPathInsideBlackout()
            time.sleep(3)
    except Exception, err:
        print "Got Exception", str(err)
        traceback.print_exc()
    finally:
        print "Disconnecting"
        dce_ctrl.Disconnect()
