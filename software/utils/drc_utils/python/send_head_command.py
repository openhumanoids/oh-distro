#!/usr/bin/python
import os,sys
import time

home_dir =os.getenv("DRC_BASE")
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import math
import numpy as np
import lcm
from bot_core.pose_t import pose_t
import time
import drc_utils as bot

pitch = 5.0
yaw = 10.0
pitch_rad = pitch*np.pi/180.0
yaw_rad =yaw*np.pi/180.0

lc = lcm.LCM()
print "Send DESIRED_HEAD_ORIENTATION..."
print pitch , pitch_rad
print yaw , yaw_rad
msg = pose_t();
msg.utime = bot.timestamp_now()
msg.orientation = bot.euler_to_quat([0, pitch_rad,  yaw_rad ])
lc.publish("DESIRED_HEAD_ORIENTATION", msg.encode())

