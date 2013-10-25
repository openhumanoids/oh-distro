#!/usr/bin/env python

import roslib; roslib.load_manifest('handle_collisions')
import rospy

from sensor_msgs.msg import JointState
from handle_msgs.msg import HandleSensors
from handle_msgs.msg import HandleSensorsCalibrated
from handle_msgs.msg import Finger
from handle_msgs.msg import HandleCollisions
from handle_msgs.msg import Collision

from SensorTactile import *

import math
import numpy
import string

collisions_msg = None
publisher = None
collision_observer = None

DEBUG = True

THRESHOLD = 10

class TactileArrayCollisionObserver():
	def __init__(self, frameID, data_path):
		self.frameID = frameID
		self.data_path = str(data_path)
		self.data_values = None
		self.data_values_history = None

		# this class will be used to find xy positions of the sensors .. 
		self.tactile_distal_class = TactileDistal()
		self.tactile_proximal_class = TactileProximal()
		self.tactile_palm_class = TactilePalm()

	def update_values(self,data_msg):
		# find the path of the data ...
		new_data=list()
		if (self.data_path.find('finger') > -1):
			fngr='fingerTactile'
			data=getattr(data_msg,fngr)[int(self.data_path[len(fngr)+1:len(fngr)+2])]
			fngr=self.data_path[len(fngr)+4:]
			new_data=getattr(data,fngr)
#			print new_data
		if (self.data_path.find('palm') > -1):
			fngr='palmTactile'
			new_data=getattr(data_msg,fngr)
#			fngr=self.data_path[len(fngr)+4:]
#			new_data=getattr(data,fngr)
#			print new_data
		self.data_values_history=self.data_values
		self.data_values=numpy.array(new_data)
		if (self.data_values_history==None):self.data_values_history=self.data_values

	def is_collision(self):
		#TODO: implement smart collision when time is available ... meanwhile use threshold
		#temp_data=self.data_values_history-self.data_values
		temp_data=abs(self.data_values)
		if (numpy.max(temp_data)>THRESHOLD):
			return True
		return False

	def get_collisions(self):
		collisions_map=list()
#		temp_data=self.data_values_history-self.data_values
		temp_data=abs(self.data_values)
		for i in range(len(temp_data)):
			# now move sensors by sensor and check if the values are higher than threshold ..
			if (temp_data[i]>THRESHOLD):
				#TODO: need to resolve the position of the force ..
				# lets implement for distal at the moment
				if (self.data_path.find('distal') > -1):
					(tmp_x,tmp_y)=self.tactile_distal_class.get_mapping_xy(i)
					# reorient the frame
					x=tmp_x
					y=0.01 # distance of the sensors from the surface
					z=tmp_y
				elif (self.data_path.find('proximal') > -1):
					(tmp_x,tmp_y)=self.tactile_proximal_class.get_mapping_xy(i)
					# reorient the frame
					x=tmp_x
					y=0.01 # distance of the sensors from the surface
					z=tmp_y

				elif (self.data_path.find('palm') > -1):
					(tmp_x,tmp_y)=self.tactile_palm_class.get_mapping_xy(i)
					# reorient the frame
					x=tmp_x
					y=-tmp_y
					z=0.08  # distance of the sensors from the surface
				else:
					x=0
					y=0
					z=0
				collisions_map.append([self.frameID, i, temp_data[i], x, y, z])
		return collisions_map

class HandleCollisionObserver():
	def __init__(self):

		# lets make lists of sensors ...
		# this is tactile array list 
		self.tactile_arrays_list=list()
		# the parameters are : link name for visualiser, name of the data in the collisions publisher
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[0]/proximal_link','fingerTactile[0].proximal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[0]/distal_link','fingerTactile[0].distal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[1]/proximal_link','fingerTactile[1].proximal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[1]/distal_link','fingerTactile[1].distal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[2]/proximal_link','fingerTactile[2].proximal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('finger[2]/distal_link','fingerTactile[2].distal'))
		self.tactile_arrays_list.append(TactileArrayCollisionObserver('base_link','palmTactile'))
		#TODO: add other sensors lists here
		

	def update_values(self,new_data):
		for array_item in self.tactile_arrays_list:
			array_item.update_values(new_data)

	def is_collision(self):
		reslt=False
		# first check the tactile arrays
		for array_item in self.tactile_arrays_list:
			reslt=array_item.is_collision()
			if (reslt): return True
		#TODO: add checks from other sensors here ...
		return reslt

	def get_collisions_map(self):
		result_map=[]
		for array_item in self.tactile_arrays_list:
			if (array_item.is_collision()):
				collision_items=array_item.get_collisions()
				result_map.extend(collision_items)
		return result_map


def callback(data):
	global collisions_msg
	global publisher
	global collision_observer
	# todo: do processing to find collisions ...

	collisions_msg.header.stamp = rospy.Time.now()
	collisions_msg.collisions = list()

	collision_observer.update_values(data)

	# lets say we found a collision
	if True: #collision_observer.is_collision()):
		collisions_map=collision_observer.get_collisions_map()
		for collision_item in collisions_map:
#			print collision_item
			collision_new = Collision()
			collision_new.frame_id=collision_item[0]
			collision_new.sensor_id=collision_item[1]
			collision_new.intensity=collision_item[2]
			collision_new.x=collision_item[3]
			collision_new.y=collision_item[4]
			collision_new.z=collision_item[5]
			collisions_msg.collisions.append(collision_new)
        # publish every time to indicate when collisions have stopped
        publisher.publish(collisions_msg)

#	print data
	data_path = str("fingerTactile[0].distal")
	tmp=1
	if (data_path.find('finger') > 1):
		tmp=getattr(data,'fingerTactile')[int(data_path[14:15])]

#	print len('fingerTactile')
#	print "published a message ..."

if __name__ == '__main__':
    
	""" Create a HandleCollisions object which we use to publish the current state of the collisions. """

	collisions_msg = HandleCollisions()

	# joint_state_msg.header.stamp = rospy.Time.now()
	collisions_msg.collisions = list()

	collision_observer = HandleCollisionObserver()
	
	rospy.init_node('collisions_publisher')

	publisher = rospy.Publisher('/handle/events/collisions', HandleCollisions)

	rospy.Subscriber("/handle/sensors/calibrated", HandleSensorsCalibrated, callback)

	rospy.loginfo(rospy.get_name() + " node initialized, awaiting orders ...")

	rospy.spin()

# This is how to transform points into a different coordinate frame !
# from here : http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/ua_controllers/wubble_actions/nodes/erratic_base_action.py
#    def transform_target_point(self, point):
#        self.tf.waitForTransform(self.base_frame, point.header.frame_id, rospy.Time(), rospy.Duration(5.0))
#        return self.tf.transformPoint(self.base_frame, point)
