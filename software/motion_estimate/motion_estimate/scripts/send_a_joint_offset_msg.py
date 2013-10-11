#!/usr/bin/python
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
from drc.atlas_state_t import atlas_state_t
from drc.force_torque_t import force_torque_t
import time

msg = atlas_state_t()
# todo: use bot time:
msg.utime = 0
#msg.desired_controller_period_ms = 0



#struct atlas_state_t
#{
#  int64_t utime;

msg.num_joints =28
msg.joint_name =["null"]*msg.num_joints
msg.joint_position = [0]*msg.num_joints
msg.joint_velocity = [0]*msg.num_joints
msg.joint_effort = [0]*msg.num_joints
ft_msg = force_torque_t()
msg.force_torque = ft_msg

# Neck Angle Offset Function:
# at          add
# 0.36512     0.012
# 0.53969     0.012
# 0.71423     0.012
# 0.88868     0.006
# 1.06337     0.000
# 1.15035     -0.0075

j_body = [0, 0, 0, 0.0]
j_l_leg = [0, 0, 0, 0, 0, 0]
j_r_leg = [0, 0, 0, 0, 0, 0]
j_l_arm = [0, 0, 0, 0, 0, 0]
j_r_arm = [0, 0, 0, 0, 0, 0]

#j_body = [0, 0, 0, 0]
#j_l_leg = [0, 0, 0, 0, 0, 0]
#j_r_leg = [0, 0, 0, 0, 0, 0]
#j_l_arm = [0, 0, 0, 0, 0, 0]
#j_r_arm = [-0.005, 0.09, 0.00, -0.02, 0, 0]
#s + backwards rotation
#s + downward (lift axis)
#uarm + thumb inwards rotate
#elbow + straighten
#larm + thumb inwards rotate
#wrist + flex away


msg.joint_position=j_body
msg.joint_position.extend(j_l_leg)
msg.joint_position.extend(j_r_leg)
msg.joint_position.extend(j_l_arm)
msg.joint_position.extend(j_r_arm)
print msg.joint_position

#  string joint_name[num_joints];
#  float joint_position [num_joints];
#  float joint_velocity [num_joints];
#  float joint_effort[num_joints];

#  force_torque_t force_torque;
#}


lc = lcm.LCM()

msg.utime = msg.utime +1
lc.publish("ATLAS_POT_OFFSETS", msg.encode())
print "ATLAS_POT_OFFSETS sent"
