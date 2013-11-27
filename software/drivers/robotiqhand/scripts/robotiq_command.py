#!/usr/bin/env python

import os,sys
home_dir = os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
import robotiqhand

import baseSModel

from time import sleep, time

def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if char == 'a':
        command = robotiqhand.command_t();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = robotiqhand.command_t();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0

    if char == 'p':
        command.rMOD = 1

    if char == 'w':
        command.rMOD = 2

    if char == 's':
        command.rMOD = 3

    #If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0


    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand  = 'Simple S-Model Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = '  + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
##    currentCommand += ', rGLV = ' + str(command.rGLV)
##    currentCommand += ', rICF = ' + str(command.rICF)
##    currentCommand += ', rICS = ' + str(command.rICS)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)

    #We only show the simple control mode
##    currentCommand += ', rPRB = ' + str(command.rPRB)
##    currentCommand += ', rSPB = ' + str(command.rSPB)
##    currentCommand += ', rFRB = ' + str(command.rFRB)
##    currentCommand += ', rPRC = ' + str(command.rPRC)
##    currentCommand += ', rSPC = ' + str(command.rSPC)
##    currentCommand += ', rFRC = ' + str(command.rFRC)
##    currentCommand += ', rPRS = ' + str(command.rPRS)
##    currentCommand += ', rSPS = ' + str(command.rSPS)
##    currentCommand += ', rFRS = ' + str(command.rFRS)

    print currentCommand

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return raw_input(strAskForCommand)

def uint_int_convert(command):

    attr_list = [x for x in dir(command) if x[0]=='r']

    for attr in attr_list:
        x = command.__getattribute__(attr)
        if x > 127:
            command.__setattr__(attr,x-256)

    return command

def publisher(side):
    """Main loop which requests new commands and publish them on the
    SModelRobotOutput topic."""

    command_topic = side.upper() + "_ROBOTIQ_COMMAND"

    lc = lcm.LCM()

    command = robotiqhand.command_t();

    try:
        while True:
            command = genCommand(askForCommand(command), command)
            command.utime = (time() * 1000000)
            command = uint_int_convert(command)

            lc.publish(command_topic, command.encode())

            sleep(0.1)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':

    if not len(sys.argv) == 2:
        print "USAGE: robotiq_command <right/left>"
        sys.exit(0)

    publisher(sys.argv[1])
