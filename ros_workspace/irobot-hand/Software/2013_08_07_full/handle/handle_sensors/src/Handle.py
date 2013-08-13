#
# Handle Class
# 
# builds a class for sensors, and does all the data manipulation (e.g. calibration)
#
# Yaroslav Tenzer May 2012

#TODO: should be split for finger class and palm class

import rospy
from handle_msgs.msg import HandleSensors
from handle_msgs.msg import Finger

import os, sys
import random
import time
import numpy

#from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum

import time

from SensorTactile import *
from JointFlexible import *
from JointBase import *
from ConfigUtilities import *

FINGER_ONE = 0
FINGER_TWO = 1
FINGER_THREE = 2

DEBUG=0
			
class FingerDataMonitor():

	def __init__(self):
		self.data_buffer_length=10
		self.data_buffer_index=0
		self.data_buffer=[numpy.random.randint(1, 50) for n in range(self.data_buffer_length)]
		self.just_back_flag=False

	def update_data(self,data):
		if (self.data_buffer_index==self.data_buffer_length):
			self.data_buffer_index=0
		self.data_buffer[self.data_buffer_index]=data
		self.data_buffer_index=self.data_buffer_index+1

	def is_detached(self):
#		print "std - ", numpy.std(self.data_buffer), self.data_buffer
		if (numpy.std(self.data_buffer)==0.0):
			self.just_back_flag=True
			return True
		return False

	def is_just_back(self):
		if (not self.is_detached()):
			if (self.just_back_flag==True):
				self.just_back_flag=False
				return True
			return False
		return False

class HandleFinger():
	def __init__(self, fingerNumberNew):
		self.fingerNumber=fingerNumberNew
		self.tactileDistal = TactileDistal()
		self.tactileProximal = TactileProximal()
		self.jointFlexible = JointFlexible()
		self.jointBase = JointBase()

		self.messageTactile = Finger()

		self.fingerDataMonitor=FingerDataMonitor()

	def calibrate(self, dataCalibrationNew):
		dataCalibration=dataCalibrationNew[self.fingerNumber]
		self.jointBase.calibrate(dataCalibration)
		self.jointFlexible.calibrate(dataCalibration)
#		self.tactileDistal.calibrate(dataCalibration)
#		self.tactileProximal.calibrate(dataCalibration)

	def re_calibrate(self):
		self.tactileDistal.re_calibrate()
		self.tactileProximal.re_calibrate()


	def updateData(self,data):
		values = array(data.fingerTactile[self.fingerNumber].distal)
		self.tactileDistal.set_values(values)

		# monitor the data on one sensor
		# this is used to check whenever finger was detached
#		self.fingerDataMonitor.update_data(values[0])

		values = array(data.fingerTactile[self.fingerNumber].proximal)
		self.tactileProximal.set_values(values)

		self.jointFlexible.set_values(data.distalJointAngle[self.fingerNumber], data.motorHallEncoder[self.fingerNumber], data.proximalJointAngle[self.fingerNumber])
		values = data.proximalJointAngle[self.fingerNumber]
		self.jointBase.set_values(values)

	def get_tactile_message(self):
		self.messageTactile.distal=self.tactileDistal.get_converted_values()
		self.messageTactile.proximal=self.tactileProximal.get_converted_values()
		return self.messageTactile

class ARMH_Handle():
	def __init__(self):

		self.fingerOne=HandleFinger(FINGER_ONE)
		self.fingerOne.tactileDistal.do_not_work_tiles=[(4,)]
		self.fingerTwo=HandleFinger(FINGER_TWO)
		self.fingerThree=HandleFinger(FINGER_THREE)
		self.fingerThree.tactileDistal.do_not_work_tiles=[(9,)]
		self.palm=TactilePalm()
		self.loadCalibrationData()
		self.fingerSpread=0;

	def loadCalibrationData(self):
		dataCalibration=[ConfigUtilities.load_config('finger0.yaml'), ConfigUtilities.load_config('finger1.yaml'), ConfigUtilities.load_config('finger2.yaml')]
		self.fingerOne.calibrate(dataCalibration)
		self.fingerTwo.calibrate(dataCalibration)
		self.fingerThree.calibrate(dataCalibration)

	def re_calibrate(self):
		self.fingerOne.re_calibrate()
		self.fingerTwo.re_calibrate()
		self.fingerThree.re_calibrate()
		self.palm.re_calibrate()

	def updateData(self,dataNew):
		self.fingerOne.updateData(dataNew)
		self.fingerTwo.updateData(dataNew)
		self.fingerThree.updateData(dataNew)
		self.palm.updateData(dataNew)
		self.fingerSpread=dataNew.fingerSpread



