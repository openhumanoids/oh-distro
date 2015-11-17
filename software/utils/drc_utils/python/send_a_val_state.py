#!/usr/bin/python
import os,sys
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.robot_state_t import robot_state_t
import time

lc = lcm.LCM()
print "Send Val..."
msg = robot_state_t();

msg.utime = 0;
msg.joint_name =['back_bkz', 'back_bky', 'back_bkx', 'neck_ay', 'neck_by', 'neck_cy', 
                 'r_arm_shz','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_uwy','r_arm_mwx','r_arm_lwy', 
                 'l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwy',
                 'r_leg_hpz','r_leg_hpx','r_leg_hpy','r_leg_kny','r_leg_aky','r_leg_akx',
                 'l_leg_hpz','l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx']

msg.num_joints =len(msg.joint_name);

msg.joint_position = [0]*msg.num_joints;
msg.joint_velocity = [0]*msg.num_joints;
msg.joint_effort = [0]*msg.num_joints;



for i in range(0,1000):
  i
  j=i/1000.0
  msg.pose.translation.x = j
  msg.pose.translation.y = j*2

  msg.joint_position[0] = j

  msg.pose.rotation.w = 1
  msg.pose.rotation.x = 0
  msg.pose.rotation.y = 0
  msg.pose.rotation.z = 0

  lc.publish("EST_ROBOT_STATE", msg.encode())
  time.sleep(0.01)
