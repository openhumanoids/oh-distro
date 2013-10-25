#
# Base class for tactile sensor
# 
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import time
import numpy

#from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum
import numpy as np


DEBUG=False


class TactileArray():
	# this class manipulates all the tactile data
	# the data is scaled such that no pressure will result with zero level

	def __init__(self, rows=None, columns=None):
		if rows == None or columns == None:
			self.rows = 2
			self.columns = 4
		else:
			self.rows = rows
			self.columns = columns
		self.data_raw = array([0 for n in range(rows*columns)])
		self.dataMinimum = [False]
		self.dataMaximum = [False]
		self.display = None		# this is to save the display module
		self.mapping = [False]		# this is to save mapping module - where the sensors are physically on the board
		self.dataCalibration = 0
		self.dataCalibrationBuffer = 0
		self.dataCalibrationLength = 20
		self.dataCalibrationIndx = 0
		self.flagCalibrated = False

		self.do_not_work_tiles = []    # this is an array to remember which tiles do not actually work ...

	def set_values(self,row,column,val):
		if ((row<=self.rows) and (column<=self.columns)):
			self.data_raw[row*self.column+column]=val
		else:
			print "ERROR: rows or colums are too big"

	def re_calibrate(self):
		self.dataCalibration = 0
		self.dataCalibrationBuffer = 0
		self.dataCalibrationLength = 20
		self.dataCalibrationIndx = 0
		self.flagCalibrated = False

	def calibrate(self):
		if (not self.flagCalibrated):
			if (self.dataCalibrationIndx==0):
				self.dataCalibrationBuffer=array([np.zeros(len(self.mapping),int) for n in range(self.dataCalibrationLength)])
			self.dataCalibrationBuffer[self.dataCalibrationIndx]=self.data_raw
			self.dataCalibrationIndx+=1
			if (self.dataCalibrationIndx==self.dataCalibrationLength):
				self.dataCalibration=np.zeros(len(self.mapping),int)
				for i in range(len(self.mapping)):
					self.dataCalibration[i]=np.mean(self.dataCalibrationBuffer[:,i])
				self.dataMinimum=[-100  for n in range(len(self.data_raw))]
				self.dataMaximum=[+100 for n in range(len(self.data_raw))]
				self.flagCalibrated=True



	def update_maxAndMin(self,val):
		if (not self.dataMinimum[0]):
			self.dataMinimum=val
		if (not self.dataMaximum[0]):
			self.dataMaximum=val
		self.dataMinimum=minimum(val,self.dataMinimum)
		self.dataMaximum=maximum(val,self.dataMaximum)

	def set_values(self,val):
		if (len(val)==(len(self.mapping))):
			self.data_raw=array(val)
			self.calibrate()
			self.update_maxAndMin(self.get_converted_values())
		else:
			print "ERROR: values array is too long or too short"

	def get_converted_values(self):
			# if the values were not calibrated yet - return zeros
			if (not self.flagCalibrated):
				return [0 for n in range(len(self.data_raw))]

			# check if some tiles are marked as not working ....
			# if there are none, then no computation is needed, just return values
			if (len(self.do_not_work_tiles)==0):
				return self.data_raw-self.dataCalibration

			# if there are dead tiles, compute the value of the dead one with respect to the neightboor values
			if (self.flagCalibrated):
				for n in range(len(self.do_not_work_tiles)):
					a=numpy.array(self.do_not_work_tiles[n])
					b=[0 for n in range(len(a))]
					for i in range(len(a[1:])):
						b[i]=self.data_raw[a[i]]-self.dataCalibration[a[i]]
					self.data_raw[a[0]]=self.dataCalibration[a[0]]+numpy.mean(b)
					# print self.data_raw[a[0]], self.data_raw[a[0]]-self.dataCalibration[a[0]], b
			return self.data_raw-self.dataCalibration

	def get_sensor_position(self, sensorID, new_mapping):
			if (not new_mapping[0]):
                                #print 'NEW'
				y = (sensorID/self.rows)%self.columns
				x = sensorID%self.rows
			else:
                                #print 'old mapping'
				y = (new_mapping[sensorID]/self.rows)%self.columns
				x = new_mapping[sensorID]%self.rows
			return (x, y)


class TactileDistal(TactileArray):
	def __init__(self):
		TactileArray.__init__(self, 2, 5)
		self.mapping = [2,3,4,5,6,7,8,9,1,0]  # this is to save mapping module - where the sensors are physically on the board
		self.dataMinimum = [400 for n in range(len(self.mapping))]
		self.dataMaximum = [1200 for n in range(len(self.mapping))]

#	def calibrate(self, dataCalibrationNew):
#		TactileArray.calibrate(self,dataCalibrationNew['Tactile']['distal'])


	def get_mapping_xy(self, sensor_id):
		rviz_mapping = [7,6,5,4,3,2,0,1,8,9]
		y_init=0.004		
		x_init=-0.003		
		y_spacing=0.007
		x_spacing=0.006
		(x, y) = self.get_sensor_position(sensor_id, rviz_mapping)
		return (x_init+x*x_spacing,y_init+y*y_spacing)


class TactileProximal(TactileArray):
	def __init__(self):
		TactileArray.__init__(self, 2, 6)
		self.mapping = [n for n in range(self.rows*self.columns)]  # this is to save mapping module - where the sensors are physically on the board
		self.dataMinimum = [400 for n in range(len(self.mapping))]
		self.dataMaximum = [1200 for n in range(len(self.mapping))]

#	def calibrate(self, dataCalibrationNew):
#		TactileArray.calibrate(self,dataCalibrationNew['Tactile']['proximal'])

	def get_mapping_xy(self, sensor_id):
		rviz_mapping = [11,10,9,8,7,6,5,4,3,2,0,1]
		y_init=0.015		
		x_init=-0.003		
		y_spacing=0.007
		x_spacing=0.007
		(x, y) = self.get_sensor_position(sensor_id,rviz_mapping)
		return (x_init+x*x_spacing,y_init+y*y_spacing)

class TactilePalm(TactileArray):
	def __init__(self):
		TactileArray.__init__(self, 10, 11)		
		# this is to save mapping module - where the sensors are physically on the board
		a20=[49, 58, 59,69,79,89,99,109,87,77,88,78,68,57,67,86,76,66,56,46,35,25,15,5,44,45,54,55,64,65]
		a40=[74,75,84,63,53,62,72,82,73,83,101,91,81,71,61,51,52,41]
		self.mapping = array(a20+a40)       
		self.dataMinimum = [400 for n in range(len(self.mapping))]
		self.dataMaximum = [1200 for n in range(len(self.mapping))]

		self.palm_tact_x = numpy.array([691.8246,645.5579,684.5193,676.4023,667.4737,658.5450,652.0515,639.8760,596.8561,  554.6480,572.5053,584.6807,579.8105,500.2643,503.5111,422.3415,418.2830,416.6596,416.6596,388.2503,357.4058,357.4058,  357.4058,354.9708,326.5614,357.4058,299.7754,356.5942,298.1520,356.5942,298.9637,  359.0292,  291.6585,  210.4889,216.9825,136.6246,130.1310,140.6830,162.5988,120.3906,78.1825,65.1953,   57.0784,   48.9614,   40.0327,31.1041,68.4421,22.9871])-712/2 #last is image size_x/2
		self.palm_tact_x=self.palm_tact_x*(0.045/335.8)   #(0.05/335.8)

		self.palm_tact_y = numpy.array([330.4544,  360.4871,  385.6497,  440.0333,  491.9819,  546.3655,  598.3140,  651.8860,  672.9901,  647.8275,  567.4696,  487.1117,  404.3187,  411.6240,  494.4170,  490.3585,  443.2801,  375.0977,  303.6684,  248.4731, 199.7713, 142.9526,   85.3222,   26.8801,  249.2848,  284.9994,  302.8567,  341.8181,  376.7211,  410.0006,  441.6567, 477.3713,  490.3585,  494.4170,  412.4357,  405.9421,  488.7351,  568.2813,  649.4509,  674.6135,  652.6977, 599.1257,  546.3655,  492.7936,  439.2216,  386.4614,  362.1105,  333.7012])-694/2 #last is image size_y/2
		self.palm_tact_y=self.palm_tact_y*(0.045/335.8) #(0.05/335.8)

	def updateData(self,data):
		values = array(data.palmTactile)
		self.set_values(values)


	def get_mapping_xy(self, sensor_id):
                x_init=0
                y_init=-0.01
		y_spacing=0.007
		x_spacing=0.007
		return (self.palm_tact_x[sensor_id]+x_init,self.palm_tact_y[sensor_id]+y_init)

