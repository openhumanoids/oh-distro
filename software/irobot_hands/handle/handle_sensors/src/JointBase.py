# Base class for base joint sensor
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import time
import math

#from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum

DEBUG=False


class JointBase():
	def __init__(self):
		self.dataLength = 1
		self.data = 0

		self.dataCalibration=0
		self.flagCalibrated=False

	def set_values(self,val):
		self.data=val

	def get_converted_values(self):
		if (self.flagCalibrated):
			# joint angle measured in radians
			result= (self.data + self.dataCalibration['zero']) / self.dataCalibration['tics_per_degree']
			return math.radians(result) # convert from degrees to radians
		else:
			if (DEBUG): print "ERROR: No calibration was applied to :", self.__class__.__name__
			return 0

	def calibrate(self, dataCalibrationNew):
		self.dataCalibration=dataCalibrationNew['proximalJointAngle']
		self.flagCalibrated=True

