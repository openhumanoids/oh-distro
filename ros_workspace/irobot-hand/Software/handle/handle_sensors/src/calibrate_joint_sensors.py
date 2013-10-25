#!/usr/bin/env python
####################################################################################
#
#  Purpose: calibrate tendon length and flexion deformation sensors
#
#  To Use:
#
#  Start the hand open and in the 'power grasp' configuration with the calibration
#    fixture on the palm, and the normal vector to the palm pointed up against gravity
#  
#  Ensure that the fingers are completely open
#
#  Hit play, and watch the fingers close agains the jig, curl over, and then open
#
#  Note that this will overwrite the finger<i>.yaml files
#
#  Leif Jentoft, Fall 2012

# segment motion into
# 1.) zero point (when does the finger start moving, signaling all tendon slack is taken in)
# 2.) encoder motion (after an initial motion from the distal link, the pin joint moves first)
# 3.) fingertip motion (after the pin joint closes against the jig, only the tip can move)

import roslib; roslib.load_manifest('handle_sensors')
import rospy
import numpy as np
import os, inspect

from handle_msgs.msg import HandleSensors, HandleControl
import matplotlib.pyplot as plt # used to show results of fitting
plt.ion() # turn on interactive mode so that plotting is non-blocking

# Configuration parameters
TICS_PER_RADIAN = 4200 * np.pi/2 		# how many counts does it take for the finger to reach this angle
FINGER_KPIN_START = 1000/TICS_PER_RADIAN 	# start of pin-only motion (in accelerometer angle against gravity)
FINGER_KPIN_END = 4000/TICS_PER_RADIAN 		# end of pin-only motion
FINGER_KTIP_START = 4500/TICS_PER_RADIAN 	# start of tip-only motion
FINGER_KTIP_END = 5500/TICS_PER_RADIAN 		# end of tip-only motion

FINGER_TEST = 0	# index of finger that is tested for arrival at position
POWER_GRASP = 0 # rotation corresponding to power grasp

# Set buffer size large enough it does not overflow
TOTAL_TRIAL_TIME = 30	# seconds
SAMPLING_RATE = 100 	# Hz sensor sampling rate

class DataLogger:
	"""scrape time, motor_ticks, pin_radians, tip_radians, voltages from HandleSensor message"""
	def __init__(self):
		rospy.Subscriber('/handle/sensors/raw', HandleSensors, self.callback)	
		self.i = -1
		samples = TOTAL_TRIAL_TIME * SAMPLING_RATE
		self.time = np.zeros(samples)
		self.motor_ticks = [np.zeros(samples), np.zeros(samples), np.zeros(samples)]  
		self.pin_radians = [np.zeros(samples), np.zeros(samples), np.zeros(samples)]  
		self.tip_radians = [np.zeros(samples), np.zeros(samples), np.zeros(samples)]  
		self.flexure_voltages = [np.zeros([8, samples]), np.zeros([8, samples]), np.zeros([8, samples])]

	def callback(self, data):
		self.i += 1
		if self.i >= TOTAL_TRIAL_TIME * SAMPLING_RATE:
			# print 'buffer full, no longer gathering data'
			return
		self.time[self.i] = data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs
		for i in range(3):
			self.motor_ticks[i][self.i] = data.motorHallEncoder[i]
			self.pin_radians[i][self.i] = 2 * np.pi / 1024. * data.proximalJointAngle[i]  # 1024 counts per revolution
			self.tip_radians[i][self.i] = -np.arctan2(data.distalAcceleration[i].y, data.distalAcceleration[i].z)
			self.flexure_voltages[i][:, self.i] =  data.distalJointAngle[i].proximal + data.distalJointAngle[i].distal


class CommandHand:
	"""command hand to go to given position, specified in encoder tics"""
	def __init__(self):
		self.pub = rospy.Publisher('/handle/control', HandleControl)

	def move(self, target):
		target = np.array(target) * TICS_PER_RADIAN
		self.pub.publish([HandleControl.POSITION]*5, target, [True, True, True, False, False])
	
	def move_slowly(self, start, end):
		steps = 10
		start = np.array(start) * TICS_PER_RADIAN
		end = np.array(end) * TICS_PER_RADIAN
		step = (end - start) / steps
		for i in range(steps):
			self.pub.publish([HandleControl.POSITION]*5, start + step * i, [True, True, True, False, False])
			rospy.sleep(1)

def wait_till_hand_reaches(data, tip_angle):
	"""used instead of CommandHand for passive calibration"""
	finger_index = FINGER_TEST
	iold = 0
	print 'waiting for finger%i angle to reach %f'%(finger_index, tip_angle)
	while not rospy.is_shutdown():
		if iold != data.i: print data.tip_radians[finger_index][data.i]
		iold = data.i
		if data.tip_radians[finger_index][data.i] > tip_angle:
			print 'arrived'
			return

def feedback_fit(xx, yy, title='', order=1):
	"""fit curve and show results to see if they make sense"""
	poly_coeff = np.polyfit(xx, yy, order)
	plt.clf()
	plt.plot(xx, yy, xx, np.polyval(poly_coeff, xx))
	plt.title(title)
	raw_input("press 'Enter' to proceed")	
	return poly_coeff	

def main():
	# goal: find:
	#	zero point for tendon
	#	gain for encoder spool (w.r.t. tip angle)
	#	gain for encoder pin (w.r.t. encoder spool)

	# initialize empty arrays
	zero = [0, 0, 0]
	k_pin_enc = [0.0, 0.0, 0.0]
	k_motor_enc = [0.0, 0.0, 0.0]

	# initialize hand and logger
	data = DataLogger()
	hand = CommandHand()
	rospy.init_node('calibrate_fk')
	print('please ensure:\n\t* palm is pointing up (gravity is normal to palm)\n\t* fingers start fully open\n\t* calibration jig is on palm\n\t* tendon length is roughly zeroed')
	keep_going = raw_input("hit 'Enter' to proceed, or 'q' to cancel\n")
	if keep_going != '': 
		print 'cancelling'
		return
	hand.move([FINGER_KPIN_START, FINGER_KPIN_START, FINGER_KPIN_START, 0, POWER_GRASP])
	
	# fingers start at rest angle
	pin_rest_angle = np.zeros(3)
	for i in range(3):
		pin_rest_angle[i] = data.pin_radians[i][data.i]

	# get data chunk while only pin joint moves (to calibrate base joint contribution to tendon length) 
	#wait_till_hand_reaches(data, FINGER_KPIN_START)
	rospy.sleep(1)
	start_index_k_pin_enc= data.i
	hand.move_slowly([FINGER_KPIN_START, FINGER_KPIN_START, FINGER_KPIN_START, 0, POWER_GRASP], [FINGER_KPIN_END, FINGER_KPIN_END, FINGER_KPIN_END, 0, POWER_GRASP])
	#wait_till_hand_reaches(data, FINGER_KPIN_END)
	end_index_k_pin_enc = data.i

	# get data chunk while only tip moves (to calibrate tip joint contribution to tendon length)
	#hand.move([FINGER_KTIP_START, FINGER_KTIP_START, FINGER_KTIP_START, 0, POWER_GRASP])
	#wait_till_hand_reaches(data, FINGER_KTIP_START)
	start_index_k_motor_enc = data.i
	hand.move_slowly([FINGER_KTIP_START, FINGER_KTIP_START, FINGER_KTIP_START, 0, POWER_GRASP], [FINGER_KTIP_END, FINGER_KTIP_END, FINGER_KTIP_END, 0, POWER_GRASP])	
	#wait_till_hand_reaches(data, FINGER_KTIP_END)
	end_index_k_motor_enc = data.i

	hand.move([0,0,0,0,0]) # return hand to rest position

	plt.clf()
	# plot pin angle and tip angle
	i = data.i
	plt.plot(data.time[0:i], data.pin_radians[FINGER_TEST][0:i], '-*', data.time[0:i], data.tip_radians[FINGER_TEST][0:i], '-*')
	raw_input('press Enter to proceed')			
	
	# liner fit to get k_pin_enc
	for i in range(3):
		print 'fitting finger%i: k_pin_enc'%i
		xx = data.pin_radians[i][start_index_k_pin_enc:end_index_k_pin_enc] 
		yy = data.motor_ticks[i][start_index_k_pin_enc:end_index_k_pin_enc]
		k_pin_enc[i] = feedback_fit(xx, yy, title='k_pin_enc', order=1)[0]

	# linear fit to get k_motor_enc
	index = data.i
	for i in range(3):
		print 'fitting finger%i: k_motor_enc'%i
		xx = data.motor_ticks[i][start_index_k_motor_enc:end_index_k_motor_enc]
		yy = data.tip_radians[i][start_index_k_motor_enc:end_index_k_motor_enc]
		k_motor_enc[i] = feedback_fit(xx, yy, title='k_motor_enc', order=1)[0]
	
	# get offset for cable length (this changes when hand is rezeroed or recabled)
	for i in range(3):
		zero[i] = np.mean(-pin_rest_angle[i] + k_motor_enc[i] * (data.motor_ticks[i][0:index] - k_pin_enc[i] * data.pin_radians[i][0:index]) - 
			(data.tip_radians[i][0:index] - data.pin_radians[i][0:index]))/k_motor_enc[i]

	# voltage calibration
	tip_angle = ['','','']
	flexure_coeffs = ['','','']
	for i in range(3):
		flexure_coeffs[i] = ['','','','']
		tip_angle[i] = data.tip_radians[i][0:index]- data.pin_radians[i][0:index] - pin_rest_angle[i] # could add lowpass filtering to improve this
		small_angle = np.nonzero(tip_angle[i] < 20 * np.pi / 180.)
		# plt.plot(tip_angle[i])
		# raw_input('press Enter to proceed')
		# plt.clf()
		volts = data.flexure_voltages[i]
		for j in range(4):
			if len(small_angle) > 0:
				ratios = (volts[2*j][0:index] - volts[2*j+1][0:index])/(volts[2*j][0:index] + volts[2*j+1][0:index])
				print 'fitting finger %i, ratio %i'%(i,j)
				a = np.polyfit(ratios[small_angle], tip_angle[i][small_angle], 1) 
				#plt.plot(ratios, tip_angle[i], ratios, np.polyval(a, ratios))
				flexure_coeffs[i][j] = a
			else:
				print 'no angles match fitting critera'
		#raw_input("press 'Enter' to proceed")
			
	# plot error to see if it looks acceptable
	# plt.ion() # turn on interactive mode
	#for i in range(3):	
		#plt.clf()
		#plt.plot(180/np.pi * (-pin_rest_angle[i] + k_motor_enc[i] * (data.motor_ticks[i][0:index] - k_pin_enc[i]* data.pin_radians[i][0:index] - zero[i]) - 
		#	(data.tip_radians[i][0:index] - data.pin_radians[i][0:index])), '*')
		#plt.xlabel('sample')
		#plt.ylabel('error (degrees)')
		#plt.show()
		#raw_input("press 'Enter' to proceed")

	# move there, zero tendon
	print "calibration complete.  hit 'Enter' to copy the values into finger<i>.yaml or 'q' to cancel\n"
	keep_going = raw_input()
	if keep_going != '':
		print 'cancelling and exciting'
		return

	# some messiness here to get proper config files when called from another directory
	cmd_folder = os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])
	header = open(os.path.join(cmd_folder, 'finger_yaml_header.yaml'))
	head = header.readlines()
	header.close()	
	for i in range(3):
		config = open(os.path.join(cmd_folder, 'finger%i.yaml'%i), 'w')
		for line in head:
			config.write(line)
		config.write("    k_motor_enc    : %f\n"%k_motor_enc[i])
		config.write("    motor_enc_zero : %f\n"%zero[i])
		config.write("    k_pin_enc      : %f\n"%k_pin_enc[i])
		for j in range(4):
			config.write("    %i : {a: %f, ktwist: 0.5, b: %f, kflex: 0.5}\n"%(j, flexure_coeffs[i][j][0], flexure_coeffs[i][j][1]))

	
main()
