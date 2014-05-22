#!/usr/bin/env python

import os,sys
home_dir = os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
import robotiqhand

import pyrobotiqhand.baseSModel as baseSModel

from time import sleep, time

forceLevel = 128
speedLevel = 128

def genCommand(char):
    """Update the command according to the character entered by the user."""

    global forceLevel
    global speedLevel

    command = robotiqhand.command_t();

    #this is the default, overwrite below
    command.activate = 1
    command.do_move = 1
    command.mode = -1

    if char == 'a':
        command.do_move = 0

    elif char == 'r':
        command.activate = 0
        command.do_move = 0

    elif char == 'c':
        command.ifc = 1
        command.positionA = 254
        command.positionB = 254
        command.positionC = 254
        command.force = forceLevel
        command.velocity = speedLevel

    elif char == 'o':
        command.ifc = 1
        command.positionA = 0
        command.positionB = 0
        command.positionC = 0
        command.force = forceLevel
        command.velocity = speedLevel

    elif char == 'b':
        command.do_move = 0
        command.mode = 0
        command.position = 0
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'p':
        command.do_move = 0
        command.mode = 1
        command.position = 0
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'w':
        command.do_move = 0
        command.mode = 2
        command.position = 0
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 's':
        command.do_move = 0
        command.mode = 3
        command.position = 0
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'f':
        command.do_move = 0
        speedLevel += 32
        if speedLevel > 255:
            speedLevel = 255
        print "speed level now:", speedLevel
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'l':
        command.do_move = 0
        speedLevel -= 32
        if speedLevel < 0:
            speedLevel = 0
        print "speed level now:", speedLevel
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'i':
        command.do_move = 0
        forceLevel += 32
        if forceLevel > 255:
            forceLevel = 255
        print "force level now:", forceLevel
        command.velocity = speedLevel
        command.force = forceLevel

    elif char == 'd':
        command.do_move = 0
        forceLevel -= 32
        if forceLevel < 0:
            forceLevel = 0
        print "force level now:", forceLevel
        command.velocity = speedLevel
        command.force = forceLevel

    elif char in [str(x) for x in range(254)]:
        command.ifc = 1
        command.positionA = int(char)
        command.positionB = int(char)
        command.positionC = int(char)
        command.force = forceLevel
        command.velocity = speedLevel

    elif char == 'e':
        command.emergency_release = 1

    else:
        # None of the individual commands were received, do nothing
        return None

    return command


def askForCommand():
    """Ask the user for a command to send to the gripper."""

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'e: Emergency Release\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-254): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    try:
        return raw_input(strAskForCommand)
    except EOFError:
        return ''

def publisher(side):
    """Main loop which requests new commands and publish them on the
    SModelRobotOutput topic."""

    command_topic =  "ROBOTIQ_" + side.upper() + "_COMMAND"

    lc = lcm.LCM()

    try:
        while True:
            command = genCommand(askForCommand())

            if command:
                command.utime = (time() * 1000000)
                lc.publish(command_topic, command.encode())
            else:
                print "ERROR: bad command"

            sleep(0.1)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':

    if not len(sys.argv) == 2:
        print "USAGE: robotiq_command <right/left>"
        sys.exit(0)

    publisher(sys.argv[1])
