#!/usr/bin/env python

# Created on Oct 30, 2013
# 
# @author: Twan, Maurice



import roslib; roslib.load_manifest('mit_helios_scripts')
import argparse, sys
import rospy
import lcm
import threading

from irobothand.calibrate_t import calibrate_t
from irobothand.current_control_close_t import current_control_close_t
from irobothand.position_control_close_t import position_control_close_t
from irobothand.spread_t import spread_t

calibrate_channel = 'CALIBRATE'
current_control_channel = 'CURRENT_CONTROL_CLOSE'
position_control_channel = 'POSITION_CONTROL_CLOSE'
spread_channel = 'SPREAD'

from IRobotHandController import IRobotHandController

def validIndices(validArray):
    return [i for i, valid in enumerate(validArray) if valid]

def calibrate_callback(controller, channel, message):
    if isinstance(message, str):
        message = calibrate_t.decode(message)
    controller.calibrate_motor_encoder_offsets(message.in_jig)
    controller.zero_current()

def current_control_close_callback(controller, channel, message):
    if isinstance(message, str):
        message = current_control_close_t.decode(message)
    indices = validIndices(message.valid)
    controller.close_hand_current_control(message.current_milliamps, indices)
    controller.zero_current()

def position_control_close_callback(controller, channel, message):
    if isinstance(message, str):
        message = position_control_close_t.decode(message)
    indices = validIndices(message.valid)
    controller.motor_excursion_control_close_fraction(message.close_fraction, indices)
    controller.zero_current()

def spread_callback(controller, channel, message):
    if isinstance(message, str):
        message = spread_t.decode(message)
    controller.spread_angle_control(message.angle_radians)
    controller.zero_current()

def parseArguments():
    sys.argv = rospy.myargv(sys.argv) # get rid of additional roslaunch arguments
    parser = argparse.ArgumentParser(description='Script for interacting with iRobot hand')
    parser.add_argument('side', help='hand side, l/L or r/R')
    args = parser.parse_args()
    side = args.side.lower()
    if side not in ['r', 'l']:
        raise RuntimeError("Side not recognized: " + side)
    return (side)

def get_lcm_channel(side, topic):
    fulltopic = 'IROBOT_' + side + '_' + topic
    return fulltopic

if __name__ == '__main__':
    side = parseArguments()
    controller = IRobotHandController(side)
    lock = threading.Lock()
    
    def create_callback(callback):
        def ret(channel, message):
            with lock:
                callback(controller, channel, message)
        return ret
    
    lc = lcm.LCM()
    lc.subscribe(get_lcm_channel(side, calibrate_channel), create_callback(calibrate_callback))
    lc.subscribe(get_lcm_channel(side, current_control_channel), create_callback(current_control_close_callback))
    lc.subscribe(get_lcm_channel(side, position_control_channel), create_callback(position_control_close_callback))
    lc.subscribe(get_lcm_channel(side, spread_channel), create_callback(spread_callback))
    
    while True:
        lc.handle()
