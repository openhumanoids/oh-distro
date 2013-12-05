#!/usr/bin/env python

import os,sys
home_dir = os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm

import pyrobotiqhand.baseSModel as baseSModel
import pyrobotiqhand.comModbusTcp as comModbusTcp

from time import sleep

def mainLoop(side, address):

    command_topic = "ROBOTIQ_" + side.upper() + "_COMMAND"
    status_topic = "ROBOTIQ_" + side.upper() + "_STATUS"

    #Gripper is a S-Model with a TCP connection
    gripper = baseSModel.robotiqBaseSModel()
    gripper.client = comModbusTcp.communication()

    #We connect to the address received as an argument
    gripper.client.connectToDevice(address)

    # Start LCM
    lc = lcm.LCM()

    #The Gripper command is received from the topic named 'SModelRobotOutput' now ROBOTIQ_COMMAND
    command_sub = lc.subscribe(command_topic, gripper.refreshCommand)
    # NOTE: the callback function here is just the raw handler for modbus commands, it may make
    # more sense for the message to have human readable data and have the lcm callback
    # convert to the modbus data.

    #We loop
    try:
        while True:
            lc.handle()

            #Get and publish the Gripper status
            #status = gripper.getStatus()

            ## TODO: fix encoding of uints in status message
            ##lc.publish(status_topic, status.encode())

            #Wait a little
            sleep(0.02)

            #Send the most recent command
            gripper.sendCommand()
            print gripper.message1
            print gripper.message2

            #Wait a little
            # gripper.sendCommand sends the raw message
            # 3 times, then mods it and sends that 3 times
            # each with a 1ms pause, which will take 6ms
            sleep(0.08)

    except KeyboardInterrupt:
        pass

    lc.unsubscribe(command_sub)


if __name__ == '__main__':

    #TODO: Add verification that the argument is an IP address
    if not len(sys.argv) == 3:
        print "USAGE: robotiq_tcp_node <right/left> <ip address>"
        sys.exit(0)
    mainLoop(sys.argv[1], sys.argv[2])

