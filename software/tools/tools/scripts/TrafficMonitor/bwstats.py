#!/usr/bin/env python

_description = '''
This program uses LCM to listen to the bandwidth stats messages published
by the drc-network-shaper program.  The bandwidth stats messages are published
on the channels BASE_BW_STATS and ROBOT_BW_STATS.

This program will calculate kbps averages per for each reported sent/received
channel in the bandwidth stats message.  That is, it monitors either the sent
or received traffic, on either the base or robot side.

This tool is meant to be used with the signal_scope tool.  You should launch
signal_scope and load the correct configuration file to see the plots.

Usage example with signal scope:


python bwstats.py --side robot --type sent

signal_scope $DRC_BASE/software/config/signal_scope_configs/bandwidth_stats_robot.json


Note, there are two configuration files, bandwidth_stats_robot.json and
bandwidth_stats_base.json.  Each config file contains a different set of
channels to be plotted. The base set is the channels sent by the base
side and received on the robot side.  The robot set is the channels sent
by the robot side and received on the base side.
'''

import sys
import json
import time
import os
import argparse
import drc as lcmdrc
import lcmutils
import numpy as np
import pprint
import colorlist
import signal
from utime import getUtime
from simpletimer import SimpleTimer, MovingAverageComputer, AverageComputer




############################
BASE = 'BASE'
ROBOT = 'ROBOT'
SENT = 'sent'
RECEIVED = 'received'

############################
side = ROBOT
plotType = SENT
############################


class BandwidthStats(object):

    def __init__(self):
        self.averages = {}
        self.receivedBytes = {}

stats = BandwidthStats()


def getChannelSuffix():

    suffixCodes = { ROBOT : { SENT : 'R', RECEIVED : 'B'},
                    BASE  : { SENT : 'B', RECEIVED : 'R'}}

    return '_BWP' + suffixCodes[side][plotType]


def publishStats(channel, kbps):

    msg = lcmdrc.drill_control_t()
    msg.utime = getUtime()
    msg.data = [kbps]
    msg.data_length = len(msg.data)
    lcm = lcmutils.getLCM()
    lcm.publish(channel + getChannelSuffix(), msg)


def onMessage(channel, message):

    if False:
        print '-----------------------------'
        print 'received message on channel:', channel
        print type(message)
        print len(message)
        print lcmutils.getMessageFingerprint(message)
        print lcmdrc.map_image_t._get_packed_fingerprint()
        print lcmutils.getMessageFingerprint(message) == lcmdrc.map_image_t._get_packed_fingerprint()


    receivedBytesDelta = len(message)
    if channel not in stats.receivedBytes:
        stats.receivedBytes[channel] = receivedBytesDelta
    else:
        stats.receivedBytes[channel] += receivedBytesDelta
    receivedBytes = stats.receivedBytes[channel]

    bytesPerSecond = updateChannelBytes(channel, receivedBytesDelta)
    kbps = bytesPerSecond * (8/1000.0)
    print channel, receivedBytes, receivedBytesDelta, '%.2f' % bytesPerSecond, '%.2f' % kbps

    publishStats(channel, kbps)


def updateChannelBytes(channel, bytes):

    averageComputer = stats.averages.get(channel)
    if not averageComputer:
        #averageComputer = MovingAverageComputer()
        #averageComputer.timeWindow = timeWindow
        averageComputer = AverageComputer()
        stats.averages[channel] = averageComputer

    averageComputer.update(bytes)
    bytesPerSecond = averageComputer.getAverage()
    averageComputer.reset()
    return bytesPerSecond


def onBandwidthStats(channel, message):

    # sent_channels or received_channels
    channelList = getattr(message, '%s_channels' % plotType)
    bytesList = getattr(message, '%s_bytes' % plotType)

    # to generate json code for signal scope tool:
    #generatePlotsJson(channelList)
    #sys.exit(0)

    totalBytesDelta = 0

    for channel in channelList:

        channelIndex = channelList.index(channel)
        receivedBytes = bytesList[channelIndex]

        previousReceivedBytes = stats.receivedBytes.setdefault(channel, receivedBytes)
        stats.receivedBytes[channel] = receivedBytes
        receivedBytesDelta = receivedBytes - previousReceivedBytes

        totalBytesDelta += receivedBytesDelta

        bytesPerSecond = updateChannelBytes(channel, receivedBytesDelta)
        kbps = bytesPerSecond * (8/1000.0)
        #print channel, receivedBytes, receivedBytesDelta, '%.2f' % bytesPerSecond, '%.2f' % kbps
        print channel, '%.2f kbps' % kbps

        publishStats(channel, kbps)


    totalChannel = 'TOTAL'
    totalBytesPerSecond = updateChannelBytes(totalChannel, totalBytesDelta)
    totalKbps = totalBytesPerSecond * (8/1000.0)
    publishStats(totalChannel, totalKbps)

    print
    print 'Total kilobits: %.2f' % (totalBytesDelta * (8/1000.0))
    print totalChannel, '%.2f kbps' % totalKbps
    print '--------------------------------------'


def generatePlotsJson(channelList):

    plots = {}
    signals = []
    for channel in channelList:
        signal = {
          "color": colorlist.getNextColor(), 
          "messageType": "drc::drill_control_t", 
          "visible": True, 
          "fieldName": "data", 
          "arrayKeys": [
            "0"
          ], 
          "channel": channel + getChannelSuffix()
        }
        signals.append(signal)

    print
    print json.dumps(signals, indent=2, sort_keys=True)
    print


def main():

    # allow control-c to kill the program
    signal.signal(signal.SIGINT, signal.SIG_DFL)


    parser = argparse.ArgumentParser(description=_description, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-s", "--side", type=str, choices=['base', 'robot'],
                         help='on which side of the network are you running the script', required=True)

    parser.add_argument("-t", "--type", type=str, choices=['sent', 'received'],
                         help='which type of traffic would you like to monitor', required=True)

    args = parser.parse_args()

    global side, plotType
    side = args.side.upper()
    plotType = args.type

    channel = '%s_BW_STATS' % side
    lcm = lcmutils.getLCM()
    lcm.subscribe(channel, onBandwidthStats, lcmdrc.bandwidth_stats_t)
    lcm.startHandleLoop()

    # to monitor individual message channels (without using shaper bw stats)
    # this code is out of date...
    #lcm.subscribe('MAP_DEPTH', onMessage)


if __name__ == '__main__':
    main()
