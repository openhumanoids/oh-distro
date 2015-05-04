#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from ipab.hand_pose_packet_message_t import hand_pose_packet_message_t

def timestamp_now (): return int (time.time () * 1000000)

msg = hand_pose_packet_message_t()
msg.utime = timestamp_now()
msg.reference_frame = 0 # not used with joints
msg.position = [0,0,0] # ee position and orientation
msg.orientation = [1,0,0,0]
msg.robot_side = 1
msg.trajectory_time = 2

if len(sys.argv)>1:
  cmd = sys.argv[1]
  msg.data_type =  1 # ee pos=0 | joint angles =1
  msg.to_home_position = True
  msg.n_angles = 0
  msg.joint_angles = []

else:
  print 'No mode specified - sending null config'
  msg.data_type =  1 # ee pos=0 | joint angles =1
  msg.to_home_position = False
  msg.n_angles = 7;
  # left arm set point:
  msg.joint_angles = [0.2705597,-1.33021509, 2.099817,  0.50086, 0.02374, 0.00058, 1]
  #right arm set point:
  # msg.joint_angles = [0.2705597, 1.33021509, 2.099817, -0.50086, 0.02374, 0.00058]

lc = lcm.LCM()
lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
print "Commanding arm"





