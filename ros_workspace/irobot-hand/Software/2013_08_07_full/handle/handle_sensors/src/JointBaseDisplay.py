# Base class for base joint sensor
# Yaroslav Tenzer, May 2012

import rospy
from handle_msgs.msg import HandleSensors

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import pygame
import time
import math

from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum

DEBUG=False

class JointBaseDisplay():
	def __init__(self):
		self.data=0
		self.pixel_size=20
		self.screen=None
		self.position_x=0
		self.position_y=0
		self.width=120
		self.hight=40
		# pick a font you have and set its size
		self.myfont = pygame.font.SysFont("Comic Sans MS", 20)
		self.labelPosition = (50,10)
	
	def set_screen(self,screenNew):
		self.screen=screenNew

	def set_position(self,x, y):
		self.position_x=x
		self.position_y=y
		
	def draw_init(self):
		self.data=0
		self.draw()

	def draw(self):
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([0, 0, 0]), rect,0)
		rect = (self.position_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([200, 200, 200]), rect,2)
		# apply it to text on a label
		label = self.myfont.render(str(round(self.data,2)), 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = self.labelPosition
		self.screen.blit(label, (x+self.position_x, y+self.position_y))

	def updateData(self, new_data):
		self.data=new_data

