# processing pipeline to convert joint-angle-sensor readings to joint transform
import numpy as np
from ConfigParser import SafeConfigParser

# phototransistor readings to flex / twist numbers:
# 
# first, take the ratio of the difference to the sum for each pair:
#    raw_ratio = (phototransistor_1 - phototransistor_2) / (phototransistor_1 + phototransistor_2)
# then, do a first-order calibration to this
#    calibrated_ratio = a * raw_ratio + b
# finally, take the sum and difference and multiply:
#    flex_1 = k_flex * (calibrated_ratio_1 + calibrated_ratio_2)
#    twist_1 = k_twist * (calibrated_ratio_1 - calibrated_ratio_2)


def load_config(config_file)
	# config files are set up

def raw_to_ratios(raw_values, calibration_coeffs)
	# convert raw phototransistor readings to ratios of difference to sum
	for i in range(4):
		ratios(i) = calibration_coeffs[2*i] * (raw_values(2*i) - raw_values(2*i+1) / (raw_values(2*i) + raw_values(2*i+1)) + calibration_coeffs[2*i + 1]
	return ratios

def ratios_to_flextwist(ratios)
	Kflex = 1 
	Ktwist = 2
	
	flex_proximal 	= Kflex * (ratios[0] + ratios[1])
	flex_distal 	= Kflex * (ratios[2] + ratios[3])
	twist_proximal 	= Ktwist * (ratios[0] - ratios[1])
	twist_distal	= Ktwist * (ratios[2] - ratios[3])
	 
	return (flex_proximal, flex_distal, twist_proximal, twist_distal)

def Rx(theta):
	# the R4 transform for a rotation of theta radians around the x-axis
	return np.array([[1, 0, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)], [0, 0, 0, 1]])

def Ry(theta):
	# the R4 transform for a rotation of theta radians around the y-axis
	return np.array([[np.cos(theta), 0, np.sin(theta), 0], [0, 1, 0, 0], [-np.sin(theta), 0, np.cos(theta), 0], [0, 0, 0, 1]])

def flextwist_to_transform(flextwist)

	# create series of incremental rotations
	steps = 10
	flex = np.linspace(flextwist[0], flextwist[1], steps)
	twist = np.linspace(flextwist[2], flextwist[3], steps)
	
	T = np.eye(4)
	
	for i in range(steps):
		T = T * Ry(flex[i]/steps) * Rx(twist[i]/steps)
		
	return T


