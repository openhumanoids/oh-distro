#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_publisher')

import rospy
from sensor_msgs.msg import JointState
from handle_msgs.msg import HandleSensors
from handle_msgs.msg import HandleSensorsCalibrated
from handle_msgs.msg import Finger
#from handle_msgs.msg import CableTension

from JointFlexible import *

import math

HANDLE_URDF_JOINTS=8
joint_state_msg = None
publisher = None
lastpos = 0

def callback(data):
	global joint_state_msg
	global publisher
        global lastpos
	# todo: fill in joint values then
	joint_state_msg.header.stamp = rospy.Time.now()
	joint_state_msg.position[0]=data.fingerSpread
	joint_state_msg.position[1]=data.proximalJointAngle[0]
	#joint_state_msg.position[2]=??
	joint_state_msg.position[3]=data.fingerSpread
	joint_state_msg.position[4]=data.proximalJointAngle[1]
	#joint_state_msg.position[5]=??
        if data.proximalJointAngle[2] > 0 and data.proximalJointAngle[2] < 2.8:
                joint_state_msg.position[6]=data.proximalJointAngle[2]
                lastpos = data.proximalJointAngle[2]
        else:
                joint_state_msg.position[6]=lastpos
	#joint_state_msg.position[7]=??

	# position [8]

	
	flex_twist_steps=JointFlexible.flextwist_to_steps([data.distalJointAngle[0].distal[0],data.distalJointAngle[0].distal[1], data.distalJointAngle[0].proximal[0],data.distalJointAngle[0].proximal[1]])
	flex_steps=flex_twist_steps[0]
	twist_steps=flex_twist_steps[1]
	for i in range(len(flex_steps)):
		joint_state_msg.position[8+i*2]=flex_steps[i]
		joint_state_msg.position[8+i*2+1]=twist_steps[i]

	flex_twist_steps=JointFlexible.flextwist_to_steps([data.distalJointAngle[1].distal[0],data.distalJointAngle[1].distal[1], data.distalJointAngle[1].proximal[0],data.distalJointAngle[1].proximal[1]])
	flex_steps=flex_twist_steps[0]
	twist_steps=flex_twist_steps[1]
	for i in range(len(flex_steps)):
		joint_state_msg.position[28+i*2]=flex_steps[i]
		joint_state_msg.position[28+i*2+1]=twist_steps[i]

	flex_twist_steps=JointFlexible.flextwist_to_steps([data.distalJointAngle[2].distal[0],data.distalJointAngle[2].distal[1], data.distalJointAngle[2].proximal[0],data.distalJointAngle[2].proximal[1]])
	flex_steps=flex_twist_steps[0]
	twist_steps=flex_twist_steps[1]
	for i in range(len(flex_steps)):
		joint_state_msg.position[48+i*2]=flex_steps[i]
		joint_state_msg.position[48+i*2+1]=twist_steps[i]

	publisher.publish(joint_state_msg)


	#print JointFlexible.flextwist_to_steps([data.distalJointAngle[0].distal[0],data.distalJointAngle[0].distal[1], data.distalJointAngle[0].proximal[0],data.distalJointAngle[0].proximal[1]])


if __name__ == '__main__':
    
	""" Create a JointState object which we use to publish the current state of the joints. """
	# msg = JointState()
	# todo: fill in default values and string names
	# Header header
	# string[] name
	# float64[] position
	# float64[] velocity
	# float64[] effort

	joint_state_msg = JointState()

	# joint_state_msg.header.stamp = rospy.Time.now()
	joint_state_msg.name = list()
	joint_state_msg.position = list()
	joint_state_msg.velocity = list()
	joint_state_msg.effort = list()

	joint_state_msg.name.append('finger[0]/joint_base_rotation')
	joint_state_msg.name.append('finger[0]/joint_base')
	joint_state_msg.name.append('finger[0]/joint_flex')

	joint_state_msg.name.append('finger[1]/joint_base_rotation')
	joint_state_msg.name.append('finger[1]/joint_base')
	joint_state_msg.name.append('finger[1]/joint_flex')

	joint_state_msg.name.append('finger[2]/joint_base')
	joint_state_msg.name.append('finger[2]/joint_flex')

	for i in range(HANDLE_URDF_JOINTS):
		joint_state_msg.position.append(0)
		joint_state_msg.velocity.append(0)
		joint_state_msg.effort.append(0)

	# ---- --- flexible joints addinns -----
	# position [8]
	for f_id in range(3):	
		joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_flex_from_proximal_to_1')
		joint_state_msg.position.append(0)
		joint_state_msg.velocity.append(0)
		joint_state_msg.effort.append(0)
		joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_twist_from_proximal_to_1')
		joint_state_msg.position.append(0)
		joint_state_msg.velocity.append(0)
		joint_state_msg.effort.append(0)
		for j_id in range(8):
			joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_flex_from_'+str(j_id+1)+'_to_'+str(j_id+2))
			joint_state_msg.position.append(0)
			joint_state_msg.velocity.append(0)
			joint_state_msg.effort.append(0)
			joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_twist_from_'+str(j_id+1)+'_to_'+str(j_id+2))
			joint_state_msg.position.append(0)
			joint_state_msg.velocity.append(0)
			joint_state_msg.effort.append(0)
		joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_flex_from_9_to_distal')
		joint_state_msg.position.append(0)
		joint_state_msg.velocity.append(0)
		joint_state_msg.effort.append(0)
		joint_state_msg.name.append('finger['+str(f_id)+']/flexible_joint_twist_from_9_to_distal')
		joint_state_msg.position.append(0)
		joint_state_msg.velocity.append(0)
		joint_state_msg.effort.append(0)
	# end ---- --- flexible joints addinns -----
	
	rospy.init_node('joint_publisher')

	publisher = rospy.Publisher('/joint_states', JointState)

	rospy.Subscriber("/handle/sensors/calibrated", HandleSensorsCalibrated, callback)

	rospy.spin()

