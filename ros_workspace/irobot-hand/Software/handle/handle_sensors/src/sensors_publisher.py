#!/usr/bin/env python

import roslib; roslib.load_manifest('handle_sensors')

import rospy
#from sensor_msgs.msg import JointState
#from handle_msgs.msg import HandleSensors
from handle_msgs.msg import Finger
from handle_msgs.msg import HandleSensorsCalibrated
from std_msgs.msg import String
from std_srvs.srv import Empty


import math

from Handle import *

sensors_msg = None
sensors_publisher = None
handle = None

def callback(data):
	global sensors_msg
	global sensors_publisher
	global handle

	# first, update the data ...
	handle.updateData(data)

	# todo: fill in joint values then
	sensors_msg.header.stamp = rospy.Time.now()

	sensors_msg.fingerTactile[0]=handle.fingerOne.get_tactile_message()
	sensors_msg.fingerTactile[1]=handle.fingerTwo.get_tactile_message()
	sensors_msg.fingerTactile[2]=handle.fingerThree.get_tactile_message()

	sensors_msg.palmTactile=handle.palm.get_converted_values()
	
	# finger_spread - Approx. 768 ticks to rotate the fingers 90 degrees.
	sensors_msg.fingerSpread=math.radians((handle.fingerSpread*90)/768)

	sensors_msg.proximalJointAngle[0]=handle.fingerOne.jointBase.get_converted_values()
	sensors_msg.proximalJointAngle[1]=handle.fingerTwo.jointBase.get_converted_values()
	sensors_msg.proximalJointAngle[2]=handle.fingerThree.jointBase.get_converted_values()

	sensors_msg.distalJointAngle[0]=handle.fingerOne.jointFlexible.get_angle_message()
	sensors_msg.distalJointAngle[1]=handle.fingerTwo.jointFlexible.get_angle_message()
	sensors_msg.distalJointAngle[2]=handle.fingerThree.jointFlexible.get_angle_message()

	sensors_publisher.publish(sensors_msg)

def sensors_calibrate(something):
	global handle
	
	print "sensors_publisher: handle recalibration was called ..."
	handle.re_calibrate()
	return []

if __name__ == '__main__':
    
	# create a handle class to deal with the data ...

	handle=ARMH_Handle()

	""" Create a HandleSensorsCalibrated object which we use to publish the current state of the sensors. """
	# msg = HandleSensorsCalibrated()
	# todo: fill in default values and string names
	# Header header
	# Finger[3] fingerTactile
	# float32[48] palmTactile
	# int32 fingerSpread
	# int32[3] proximalJointAngle
	# Finger[3] distalJointAngle

	sensors_msg = HandleSensorsCalibrated()

	for i in range(3):
		sensors_msg.fingerTactile[i].distal=0
		sensors_msg.fingerTactile[i].proximal=0

	for i in range(48):
		sensors_msg.palmTactile[i]=0
	
	sensors_msg.fingerSpread=0

	for i in range(3):
		sensors_msg.proximalJointAngle[i]=0

	for i in range(3):
		sensors_msg.distalJointAngle[i].distal=0
		sensors_msg.distalJointAngle[i].proximal=0

	# ---------   initialise the node   ---------- 
	rospy.init_node('handle_sensors')
	sensors_publisher = rospy.Publisher('/handle/sensors/calibrated', HandleSensorsCalibrated)
	rospy.Subscriber("/handle/sensors/raw", HandleSensors, callback)

	# ---------   create a service for calibration   ---------- 
	# rospy.Subscriber("/keyboard", String, callback_keyboard)
	s = rospy.Service('/handle/events/sensors/calibrate', Empty, sensors_calibrate)
	# ------------------- 

	rospy.spin()
