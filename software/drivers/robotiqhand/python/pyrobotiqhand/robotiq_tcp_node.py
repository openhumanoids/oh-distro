#!/usr/bin/env python

import os,sys
home_dir = os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
import select
import drc

import pyrobotiqhand.baseSModel as baseSModel
import pyrobotiqhand.comModbusTcp as comModbusTcp

from time import sleep, time

connectPublished = False
activePublished = False

def publishSystemStatus(side, lcm, status):
    global connectPublished
    global activePublished

    if connectPublished and activePublished:
        return

    msg = drc.system_status_t()
    msg.utime = (time() * 1000000)
    msg.system = 4  #provided as the system level for grippers
    msg.importance = 0
    msg.frequency = 0

    if connectPublished:
        if status and status.activated == 1:
            msg.value = side.upper() + " ROBOTIQ HAND ACTIVE: Receiving status and active"
            lcm.publish("SYSTEM_STATUS", msg.encode())
            activePublished = True
    else:
        if status:
            msg.value = side.upper() + " ROBOTIQ HAND ALIVE: Receiving status messages"
            lcm.publish("SYSTEM_STATUS", msg.encode())
            connectPublished = True

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

    timeout = 100
    p = select.poll()
    p.register(lc.fileno())

    #We loop
    try:
        while True:
            #Get and publish the Gripper status
            status = gripper.getStatus()
            publishSystemStatus(side, lc, status)
            if status:
                lc.publish(status_topic, status.encode())

            res = p.poll(timeout)
            if res:
                lc.handle()

                #Send the most recent command
                gripper.sendCommand()
                print "command processed"

                sleep(0.03)
            else:
                sleep(0.09)

    except KeyboardInterrupt:
        pass

    lc.unsubscribe(command_sub)


if __name__ == '__main__':

    #TODO: Add verification that the argument is an IP address
    if not len(sys.argv) == 3:
        print "USAGE: robotiq_tcp_node <right/left> <ip address>"
        sys.exit(0)
    mainLoop(sys.argv[1], sys.argv[2])

