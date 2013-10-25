#!/usr/bin/env python
#
# Read sensor values from hand
# Load and write sensor calibrations
#
# Leif Jentoft, May 2012

import roslib; roslib.load_manifest('sensors')
import rospy
import numpy as np
import os, inspect, time
from yaml import safe_load

from handle_msgs.msg import HandleSensors

# Some constants 
TACTILE_THRESHOLD = 10
PVDF_THRESHOLD = 10

# helper functions
def safe_load_array(string):
	# wrapper to load lists as numpy arrays
	yaml_data = safe_load(string)
	for key in yaml_data.keys():
		if type(yaml_data[key]) == type(list()):
			yaml_data[key] == np.array(yaml_data[key])
	return yaml_data

def load_config(config_file):
	# some messiness here to get proper config files when called from another directory
	cmd_folder = os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])
	config_dict = safe_load_array(open(os.path.join(cmd_folder, config_file)))
	return config_dict


# main sensor class
class Sensors():
	def __init__(self, sensor_topic, palm_calibration_file, f0_calibration_file, f1_calibration_file, thumb_calibration_file, hand_calibration_file):
	
		self.palm_cal = load_config(palm_calibration_file)
		self.finger_cal = [load_config(f0_calibration_file), load_config(f1_calibration_file), load_config(thumb_calibration_file)]
	
		self.raw_data = None

		rospy.loginfo(rospy.get_name() + ' subscribing to %s'%sensor_topic)
		rospy.Subscriber(sensor_topic, HandleSensors, self.sensor_callback)
		rospy.loginfo('done')

		rospy.init_node('HandleSensors')
		
	def sensor_callback(self, data):
		# write sensor data to internal cache, check timestamps?
		self.raw_data = data
	
	# basic access methods to get calibrated data (separate so calibration is not performed constantly)
	def get_finger_tactile_proximal(self, finger_index):
		# return unitless pressure readings with calibration offset applied
		return np.array(self.raw_data.fingerTactile[finger_index].proximal) - self.finger_cal[finger_index]['Tactile']['proximal']

	def get_finger_tactile_distal(self, finger_index):
		# return unitless pressure readings with calibration offset applied
		return np.array(self.raw_data.fingerTactile[finger_index].distal) - self.finger_cal[finger_index]['Tactile']['distal']

	def get_proximal_joint_angle(self, finger_index):
		# joint angle measured in radians
		return (self.raw_data.proximalJointAngle[finger_index] - self.finger_cal[finger_index]['proximalJointAngle']['zero']) #/ self.finger_cal[finger_index]['proximalJointAngle']['tics_per_radian']

	def get_rotation_angle(self):
		# finger rotation measured in radians
		return (self.raw_data.fingerSpread - self.hand_cal['FingerSpread']['zero']) / self.hand_cal['FingerSpread']['tics_per_radian']

	def get_distal_joint_angle(self, finger_index):
		# flexion / twist rate at both ends of joint
		raw = self.raw_data.distalJointAngle[finger_index].proximal + self.raw_data.distalJointAngle[finger_index].distal
		ratios = [0.0, 0.0, 0.0, 0.0]
		flex =  [0.0, 0.0]
		twist = [0.0, 0.0]
		for i in range(4):
			ratios[i] = self.finger_cal[finger_index]['distalJointAngle'][i]['a'] * (raw[2*i] - raw[2*i+1]) / (raw[2*i] + raw[2*i+1]) + self.finger_cal[finger_index]['distalJointAngle'][i]['b']
		for i in range(2):
			flex[i] = self.finger_cal[finger_index]['distalJointAngle'][i]['kflex']*(ratios[2*i] + ratios[2*i + 1])
			twist[i] = self.finger_cal[finger_index]['distalJointAngle'][i]['ktwist']*(ratios[2*i] + ratios[2*i + 1])		
		return np.array([flex[0], twist[0], flex[1], twist[1]])

	def get_distal_joint_transform(self, finger_index):
		flextwist = self.get_joint_angle(finger_index)
		flex = [flextwist[0], flextwist[2]]
		twist = [flextwist[1], flextwist[3]]
		
		#for i in range(10):
		#	flex(i) = 
		#	twist(i)
					
	def get_finger_pvdf_proximal(self, finger_index):
		# proximal and distal finger pvdfs	
		return np.array(self.raw_data.fingerPVDF[finger_index].proximal)
	
	def get_finger_pvdf_distal(self, finger_index):
		# proximal and distal finger pvdfs	
		return np.array(self.raw_data.fingerPVDF[finger_index].distal)

	def get_palm_tactile(self):
		# calibrated 
		return np.array(self.raw_data.palmTactile) - self.palm_cal['Tactile']

	def get_palm_pvdf(self):
		return np.array(self.raw_data.palmPVDF)
	
	# advanced access methods to return higher level data
	def in_contact(self):
		pass
		#tactile_all = self.get_palm_tactile()
		#for i in range(3):
		#	tactile_all = np.concatenate(tactile_all, self.get_finger_tactile_distal(i)
		#if (tactile_all > threshold).any():
		#	return True
		#else
		#	return False
		#return False

	def get_spatial_tactile_map(self):
		# format: [x, y, z, intensity] as per Peter Allen and Hao Pen's approach
		pass # not yet implemented


if __name__ == '__main__':
	h = Sensors('palm.yaml', 'finger.yaml', 'finger.yaml', 'finger.yaml', 'hand.yaml')
	time.sleep(1)
        rospy.spin()
	#while not rospy.is_shutdown():
#		print h.get_palm_tactile()
