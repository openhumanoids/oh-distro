#
# Base class for tactile sensor
# 
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import pygame
import time
import numpy
import math

from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum
import numpy as np

from SensorTactile import *

DEBUG=False

class TactileDisplay():
	def __init__(self, new_data_class):
		self.dataClass=new_data_class
		self.data=0
		self.data_max=[]
		self.data_min=[]
		self.pixel_size_x=60
		self.pixel_size_y=30
		self.screen=None
		self.position_x=0
		self.position_y=0
		self.colors=[(0,0,0) for n in range(len(self.dataClass.mapping))]
	
	def set_screen(self,screenNew):
		self.screen=screenNew

	def set_position(self,x, y):
		self.position_x=x
		self.position_y=y

	def set_tile(self,x,y,color):
		# x and y are zero-indexed
		rect = (x * self.pixel_size_x+self.position_x, y * self.pixel_size_y+self.position_y, self.pixel_size_x, self.pixel_size_y)
		try:
			pygame.draw.rect(self.screen, color, rect) #[val])*3
		except:
			print "WARNING: colour values are too big"+str(color)

	def convert_to_log(self, value):
		# used to scale results into log - for better representation.
		result=(math.log(value)-2.35)*80
		return result

	def get_colour_values(self):
		
		values=self.data
		valuesMin=self.data_min
		valuesMax=self.data_max

		THRESHOLD_VISUAL_LEVEL=10

		# positive will be red
		# negative should be green
		# neutral should be grey
		for i in range(len(values)):		
			if (values[i]>THRESHOLD_VISUAL_LEVEL):
				value=(((values[i])*255)/valuesMax[i])
				value=self.convert_to_log(value)
				if (value>255):
					value=255
					print 'warning: clipping value'
				if (value<0):
					value=0
				self.colors[i]=array((255,255-value,255-value),dtype = int)
				if (numpy.max(self.colors[i])>255):
					print "here we go!!"+str(value)+" "+str(self.colors[i])
			elif (values[i]<-THRESHOLD_VISUAL_LEVEL):
				value=(((-values[i])*255)/(-valuesMin[i]))
				value=self.convert_to_log(value)
				if (value>255):
					value=255
					print 'warning: clipping value'
				if (value<0):
					value=0
				self.colors[i]=array((255-value,255,255-value),dtype = int)

			else:
				self.colors[i]=(255,255,255)

		return self.colors
		
	def draw_init(self):
#		for i in range(self.data.rows*self.data.columns):
		for i in range(len(self.dataClass.mapping)):
			(x, y) = self.dataClass.get_sensor_position(i,self.dataClass.mapping)
			rect = (x * self.pixel_size_x+self.position_x, y * self.pixel_size_y+self.position_y, self.pixel_size_x, self.pixel_size_y)
			pygame.draw.rect(self.screen, ([0, 0, 0]), rect,0) #[val])*3
			pygame.draw.rect(self.screen, ([200, 200, 200]), rect,2) #[val])*3

	def draw(self):
		# set up as row major for now
		colors=self.get_colour_values()

		for i in range(len(colors)):
			(x, y) = self.dataClass.get_sensor_position(i,self.dataClass.mapping)
			self.set_tile(x,y,colors[i])

	def update_max_and_min(self,val):
		if (len(self.data_min)==0):
			self.data_min=numpy.array(val)-200
		if (len(self.data_max)==0):
			self.data_max=numpy.array(val)+200
		self.data_min=minimum(val,self.data_min)-1
		self.data_max=maximum(val,self.data_max)+1

	def updateData(self, new_data):
		self.data=new_data
		self.update_max_and_min(new_data)


class TactilePalmDisplay(TactileDisplay):

	def __init__(self, screenNew, xNew, yNew):
		TactileDisplay.__init__(self, TactilePalm())
		self.data=0
		self.screen=screenNew
		self.x=xNew
		self.y=yNew
		self.position_x=xNew
		self.position_y=yNew
		self.pixel_size_x=40
		self.pixel_size_y=40
		self.draw_init()

	def draw_init(self):
		for i in range(len(self.dataClass.mapping)):
			(x, y) = self.dataClass.get_sensor_position(i,self.dataClass.mapping)
			pos = (x * self.pixel_size_x+self.position_x, y * self.pixel_size_y+self.position_y)
			pygame.draw.circle(self.screen, (200,200,200), pos, 20, 0) #[val])*3
	def set_tile(self,x,y,color):
		# x and y are zero-indexed
		pos = (x * self.pixel_size_x+self.position_x, y * self.pixel_size_y+self.position_y)
		try:
			pygame.draw.circle(self.screen, color, pos, 20, 0) #[val])*3
		except:
			print "WARNING: colour values are too big"+str(color)



