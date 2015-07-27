#!/usr/bin/python
import os,sys
home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import math
import lcm
from bot_core.pose_t import pose_t
from bot_core.rigid_transform_t import rigid_transform_t
from drc_utils import *

msg = pose_t()
msg.utime = 0
msg.pos = (0, 0, 0.0)
msg.orientation = (1, 0, 0, 0)

lc = lcm.LCM()
#lc.publish("PRE_SPINDLE_TO_POST_SPINDLE", msg.encode())


msg = rigid_transform_t()
msg.utime = 0
msg.trans = (0, 0, 0.0)
msg.quat = euler_to_quat([0,0, math.pi])

lc = lcm.LCM()
lc.publish("PRE_SPINDLE_TO_POST_SPINDLE", msg.encode())
