#!/usr/bin/env python

import os,sys
home_dir = os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
import select
import bot_core
from time import sleep, time
from datetime import datetime


import pyrobotiqhand.baseSModel as baseSModel
import pyrobotiqhand.comModbusTcp as comModbusTcp

connectPublished = False
activePublished = False

def publishSystemStatus(side, lcm, status):
    global connectPublished
    global activePublished

    if connectPublished and activePublished:
        return

    msg = bot_core.system_status_t()
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

l_names = [ "left_finger_1_joint_1", "left_finger_1_joint_2", "left_finger_1_joint_3",
        "left_finger_2_joint_1", "left_finger_2_joint_2", "left_finger_2_joint_3",
        "left_finger_middle_joint_1", "left_finger_middle_joint_2", "left_finger_middle_joint_3",
        "left_palm_finger_1_joint", "left_palm_finger_2_joint"]
r_names = [ "right_finger_1_joint_1", "right_finger_1_joint_2", "right_finger_1_joint_3",
        "right_finger_2_joint_1", "right_finger_2_joint_2", "right_finger_2_joint_3",
        "right_finger_middle_joint_1", "right_finger_middle_joint_2", "right_finger_middle_joint_3",
        "right_palm_finger_1_joint", "right_palm_finger_2_joint"]

def get_mapping(input_val):
    return 1.0*input_val/255

def publishJointStates(side, lcm, status):
    global l_names
    global r_names

    state = bot_core.joint_state_t()
    state.utime = status.utime
    state.num_joints = 11

    state.joint_velocity = [0]*11
    state.joint_effort = [0]*11
    state.joint_position = [0]*11

    state.joint_position[0] = get_mapping(status.positionA)
    state.joint_position[1] = get_mapping(status.positionA)
    state.joint_position[2] = get_mapping(status.positionA)

    state.joint_position[3] = get_mapping(status.positionB)
    state.joint_position[4] = get_mapping(status.positionB)
    state.joint_position[5] = get_mapping(status.positionB)

    state.joint_position[6] = get_mapping(status.positionC)
    state.joint_position[7] = get_mapping(status.positionC)
    state.joint_position[8] = get_mapping(status.positionC)

    state.joint_position[9] = -0.002 * (status.positionS-137)
    state.joint_position[10] = 0.002 * (status.positionS-137)

    if (side.upper()[0] == 'R'):
        state.joint_name = r_names
        lcm.publish("ROBOTIQ_" + side.upper() + "_STATE", state.encode())
    else:
        state.joint_name = l_names
        lcm.publish("ROBOTIQ_" + side.upper() + "_STATE", state.encode())

# Create a function for periodically publishing alive signal
# Current rate is about once every 10 seconds
count = 50
def printAlive():
    global count
    count+=1
    if count > 50:
        count = 0
        print "Gripper connected.", datetime.today()

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
            if gripper:
                status = gripper.getStatus()
                if status:
                    publishSystemStatus(side, lc, status)
                    publishJointStates(side, lc, status)
                    lc.publish(status_topic, status.encode())
                    printAlive()
                else:
                    print "Gripper not found.  Trying to reconnect..."
            res = p.poll(timeout)
            if res:
                lc.handle()

                #Send the most recent command
                gripper.sendCommand()
                print "command received"

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

