#!/usr/bin/python
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
from drc import *
import time

pos = position_3d_t()
pos.translation = vector_3d_t()
pos.translation.x =0
pos.translation.y =0
pos.translation.z =0
pos.rotation = quaternion_t()
pos.rotation.w =1
pos.rotation.x =0
pos.rotation.y =0
pos.rotation.z =0

twist = twist_t()
twist.linear_velocity = vector_3d_t()
twist.angular_velocity = vector_3d_t()



msg = ee_goal_t()
msg.utime = 0
msg.ee_goal_pos = pos
msg.ee_goal_twist = twist

msg.num_chain_joints = 0

lc = lcm.LCM()
print "sent msg"
lc.publish("LEFT_PALM_GOAL", msg.encode())

