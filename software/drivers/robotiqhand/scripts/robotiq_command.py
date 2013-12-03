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

forceLevel = 128
speedLevel = 128

def genCommand(char):
    """Update the command according to the character entered by the user."""

    global forceLevel
    global speedLevel

    command = robotiqhand.command_original_t();

    if char == 'c':
        command.go_to = 1
        command.position = 255

    if char == 'o':
        command.go_to = 1
        command.position = 0

    if char == 'b':
        command.mode = 0

    if char == 'p':
        command.mode = 2

    if char == 'w':
        command.mode = 1

    if char == 's':
        command.mode = 3

    if char == 'f':
        speedLevel += 16
        if speedLevel > 255:
            speedlLevel = 255
        print "speed level now:", self.speedLevel
        command.speed = speedLevel

    if char == 'l':
        speedLevel += 16
        if speedLevel > 255:
            speedlLevel = 255
        print "speed level now:", self.speedLevel
        command.speed = speedLevel

    if char == 'i':
        forceLevel += 16
        if forceLevel > 255:
            forcelLevel = 255
        print "force level now:", self.forceLevel
        command.force = forceLevel

    if char == 'd':
        forceLevel -= 16
        if forceLevel > 0:
            command.forceLevel = 0
        print "force level now:", self.forceLevel
        command.force = forceLevel

    if char in [str(x) for x in range(255)]:
        command.go_to = 1
        command.position = int(char)

    return command


def askForCommand():
    """Ask the user for a command to send to the gripper."""

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


def publisher(side):
    """Main loop which requests new commands and publish them on the
    SModelRobotOutput topic."""

    command_topic = side.upper() + "_ROBOTIQ_COMMAND"

    lc = lcm.LCM()

    command = commandMsg()

    try:
        while True:
            command = parseString(genCommand(askForCommand()))

            command.utime = (time() * 1000000)

            lc.publish(command_topic, command.encode())

            sleep(0.1)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':

    if not len(sys.argv) == 2:
        print "USAGE: robotiq_command <right/left>"
        sys.exit(0)

    publisher(sys.argv[1])
