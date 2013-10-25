#!/usr/bin/env python
#
# Unitities to load configuration data ...
#
# Leif Jentoft, Yaroslav Tenzer May 2012

#import roslib; roslib.load_manifest('handle_sensors')
import rospy
import numpy as np
import os, inspect, time
from yaml import safe_load, dump


class ConfigUtilities():
	def __init__(self):
		pass # implement ?

	@staticmethod
	def safe_load_array(string):
		# wrapper to load lists as numpy arrays
		yaml_data = safe_load(string)
		for key in yaml_data.keys():
			if type(yaml_data[key]) == type(list()):
				yaml_data[key] == np.array(yaml_data[key])
		return yaml_data
	
	@staticmethod
	def load_config(config_file):
		# some messiness here to get proper config files when called from another directory
		cmd_folder = os.path.abspath(os.path.split(inspect.getfile( inspect.currentframe() ))[0])
		config_dict = ConfigUtilities.safe_load_array(open(os.path.join(cmd_folder, config_file)))
		return config_dict

	@staticmethod
	def write_config(config_file, section, values):
		pass # implement


