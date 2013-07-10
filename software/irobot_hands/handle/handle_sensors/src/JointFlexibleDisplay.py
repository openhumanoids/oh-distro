#
# Base class for flexible sensor
#
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors
from handle_msgs.msg import Finger

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import pygame
import time
import numpy as np

from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum

from JointFlexible import *

DEBUG=False
FONT_SIZE = 30

class JointFlexibleDisplayText():
	def __init__(self):
		self.data=0
		self.dataClass=JointFlexible()
		self.pixel_size=20
		self.screen=None
		self.position_x=0
		self.position_y=0
		self.width=120
		self.hight=80
		# pick a font you have and set its size
		self.myfont = pygame.font.SysFont("Comic Sans MS", FONT_SIZE)
		self.labelsPosition = array([[0,0] for n in range(4)])
		self.labelsPosition[self.dataClass.PROXIMAL_flex] = [ 10 , 7 ]
		self.labelsPosition[self.dataClass.PROXIMAL_twist] = [ 65 , 7 ]
		self.labelsPosition[self.dataClass.DISTAL_flex] = [ 10 , 55 ]
		self.labelsPosition[self.dataClass.DISTAL_twist] = [ 65 , 55 ]

	
	def set_screen(self,screenNew):
		self.screen=screenNew

	def set_position(self,x, y):
		self.position_x=x
		self.position_y=y
		
	def draw_init(self):
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([0, 0, 0]), rect,0)
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([200, 200, 200]), rect,2)
		data=[0,0,0,0]
		for i in range(self.dataClass.dataLength):
			# apply it to text on a label
			label = self.myfont.render(str(round(data[i],2)), 1, [255,255,255])
			# put the label object on the screen at point x=100, y=100
			(x, y) = self.labelsPosition[i]
			self.screen.blit(label, (x+self.position_x, y+self.position_y))

	def draw(self):
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([0, 0, 0]), rect,0)
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([200, 200, 200]), rect,2)
		data=self.data
		for i in range(self.dataClass.dataLength):
			# apply it to text on a label
			label = self.myfont.render(str(round(data[i],2)), 1, [255,255,255])
			# put the label object on the screen at point x=100, y=100
			(x, y) = self.labelsPosition[i]
			self.screen.blit(label, (x+self.position_x, y+self.position_y))

	def updateData(self, new_data):
		self.data=[new_data.distal[0],new_data.distal[1],new_data.proximal[0], new_data.proximal[1]]

