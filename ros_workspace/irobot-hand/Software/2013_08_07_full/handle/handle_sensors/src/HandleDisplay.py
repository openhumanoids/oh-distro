#!/usr/bin/env python
#
# tactile sensor display
# 
# subscribes to HandleSensors topic and displays sensor values
#
# based on example http://www.pygame.org/docs/ref/draw.html#pygame.draw.rect
#
# Yaroslav Tenzer May 2012

import roslib; roslib.load_manifest('handle_sensors')
import rospy

from handle_msgs.msg import *

# This is an example that uses pygame.draw.rect:
import os, sys
import random
import pygame
import time

from pygame.locals import *
from numpy import array
from numpy import minimum
from numpy import maximum

import time

from Handle import *
from SensorTactileDisplay import *
from JointFlexibleDisplay import *
from JointFlexibleDisplay import *
from JointBaseDisplay import *

from std_srvs.srv import Empty

# screen is 1080x1920
FONT_SIZE = 30

class FingerDisplay():
	def __init__(self, new_finger_number, screenNew, xNew, yNew):
		self.screen=screenNew
		self.finger_number=new_finger_number
		self.x=xNew
		self.y=yNew
		self.tactileDistalDisplay = TactileDisplay(TactileDistal())
		self.tactileDistalDisplay.set_screen(self.screen)
		self.tactileDistalDisplay.set_position(self.x+10,self.y+10)
		self.tactileDistalDisplay.draw_init()

		self.tactileProximalDisplay = TactileDisplay(TactileProximal())
		self.tactileProximalDisplay.set_screen(self.screen)
		self.tactileProximalDisplay.set_position(self.x+10,self.y+250)
		self.tactileProximalDisplay.draw_init()

		self.jointFlexibleDisplay = JointFlexibleDisplayText()
		self.jointFlexibleDisplay.set_screen(self.screen)
		self.jointFlexibleDisplay.set_position(self.x+10,self.y+165)
		self.jointFlexibleDisplay.draw_init()

		self.jointBaseDisplay = JointBaseDisplay()
		self.jointBaseDisplay.set_screen(self.screen)
		self.jointBaseDisplay.set_position(self.x+10,self.y+433)
		self.jointBaseDisplay.draw_init()

	def updateData(self,data):
		self.tactileDistalDisplay.updateData(data.fingerTactile[self.finger_number].distal)
		self.tactileProximalDisplay.updateData(data.fingerTactile[self.finger_number].proximal)
		self.jointFlexibleDisplay.updateData(data.distalJointAngle[self.finger_number])
		self.jointBaseDisplay.updateData(data.proximalJointAngle[self.finger_number])

	def draw(self):
#		if (self.finger.fingerDataMonitor.is_detached()):
#			self.tactileDistalDisplay.draw_init()
#			self.tactileProximalDisplay.draw_init()
#			self.jointFlexibleDisplay.draw_init()
#			self.jointBaseDisplay.draw_init()
#		else:
			self.tactileDistalDisplay.draw()
			self.tactileProximalDisplay.draw()
			self.jointFlexibleDisplay.draw()
			self.jointBaseDisplay.draw()

class ARMH_Handle_Display():
	def __init__(self, screenNew):
		self.screen=screenNew
		self.position_x=170
		self.position_y=0
		self.width=1920-35-600
		self.hight=1080/2-15

		self.myfont = pygame.font.SysFont("Comic Sans MS", FONT_SIZE)
		self.drawLabels()

		self.finger_0_display=FingerDisplay(0,self.screen, self.position_x+100, self.position_y+10)
		self.finger_1_display=FingerDisplay(1,self.screen, self.position_x+300, self.position_y+10)
		self.finger_2_display=FingerDisplay(2,self.screen, self.position_x+500, self.position_y+10)
		self.palmDisplay=TactilePalmDisplay(self.screen, self.position_x+660, self.position_y+30)

	def drawLabels(self):
		labels_pos_x=self.position_x-155
		labels_pos_y=self.position_y
		rect = (labels_pos_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([50, 50, 50]), rect,0)
		rect = (labels_pos_x, self.position_y, self.width, self.hight)
		pygame.draw.rect(self.screen, ([200, 200, 200]), rect,2)
		# apply it to text on a label
		label = self.myfont.render('Distal', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (10,70)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))
		label = self.myfont.render('Tactile Arrays', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (x,y+30)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

		label = self.myfont.render('Flexible Joint', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (10,200)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

		label = self.myfont.render('Proximal', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (10,300)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

		label = self.myfont.render('Tactile Arrays', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (x,y+30)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

		label = self.myfont.render('Palm Tactile Arrays', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (900-5,450)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

		label = self.myfont.render('Base Encoders', 1, [255,255,255])
		# put the label object on the screen at point x=100, y=100
		(x, y) = (10,450)
		self.screen.blit(label, (x+labels_pos_x, y+labels_pos_y))

	def updateData(self,data):
		self.finger_0_display.updateData(data)
		self.finger_1_display.updateData(data)
		self.finger_2_display.updateData(data)
		self.palmDisplay.updateData(data.palmTactile)

	def draw(self):
		self.finger_0_display.draw()
		self.finger_1_display.draw()
		self.finger_2_display.draw()
		self.palmDisplay.draw()


class PyGameDisplay():
	def __init__(self):

		pygame.init()
#		screen_size = (self.data.rows * self.pixel_size, self.data.columns * self.pixel_size)
		screen_size = (1920-600, 1080/2)
		APPLICATION_size_x = screen_size[0]
		APPLICATION_size_y = screen_size[1]
		os.environ['SDL_VIDEO_WINDOW_POS'] = '220,100'
		self.screen = pygame.display.set_mode((APPLICATION_size_x, APPLICATION_size_y))
		pygame.display.set_caption('TactileSensorDisplay')
		pygame.mouse.set_visible(True)
		#pygame.mouse.set_visible(False)
		black_square_that_is_the_size_of_the_screen = pygame.Surface(self.screen.get_size())
		black_square_that_is_the_size_of_the_screen.fill((0, 0, 0))
		self.screen.blit(black_square_that_is_the_size_of_the_screen, (0, 0))
		pygame.display.flip()

	def getScreen(self):
		return self.screen

	def flip(self):
		pygame.display.flip()



# Base Class

class DisplayNode():
	def __init__(self):

		self.pygDisplay=PyGameDisplay()
		self.handDisplay=ARMH_Handle_Display(self.pygDisplay.getScreen())
		self.pygDisplay.flip()
		self.timeLast=time.time()

		# subscribe to Korg messages, set up publishing channel for controlling hand
		rospy.init_node('HandleDisplay')

		self.init = [False]
		
		rospy.loginfo(rospy.get_name() + " subscribing to topic 'HandleSensorsCalibrated'")
		rospy.Subscriber("/handle/sensors/calibrated", HandleSensorsCalibrated, self.callback)
		rospy.loginfo(rospy.get_name() + " subscribed")

		rospy.loginfo(rospy.get_name() + " node initialized, awaiting orders ...")

		running = True

		while (not rospy.is_shutdown()) and (running):
			event = pygame.event.wait()
			if event.type == pygame.QUIT:
				running = False  # Be IDLE friendly
		        if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_c:
					print "Recalibrating ..."
					self.calibrate_sensors()
#			rospy.sleep(0.01)	
		pygame.quit()		
#		rospy.spin()
		
	def callback(self, data):
		self.handDisplay.updateData(data)

		# drawing can be made slower than data collecting	
		if ((time.time()-self.timeLast)>0.1):
			self.timeLast=time.time()
			self.handDisplay.draw()
			self.pygDisplay.flip()

	def calibrate_sensors(self):
	    rospy.wait_for_service('/handle/events/sensors/calibrate')
	    try:
		print "calling calibration service"
		service_calib=rospy.ServiceProxy('/handle/events/sensors/calibrate', Empty)
		service_calib()
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e


display = DisplayNode()

