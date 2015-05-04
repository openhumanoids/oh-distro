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


lc = lcm.LCM()

msg = hand_pose_packet_message_t()
msg.utime = timestamp_now()
msg.reference_frame = 0 # not used with joints
msg.position = [0,0,0] # ee position and orientation
msg.orientation = [1,0,0,0]
msg.trajectory_time = 2

msg.data_type =  1 # ee pos=0 | joint angles =1
msg.to_home_position = False


# left arm set point:
msg.robot_side = 0
msg.joint_angles = [0.2705597,-1.33021509, 2.099817,  0.50086, 0.02374, 0.00058,0]
msg.n_angles = len(msg.joint_angles)
lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())

time.sleep(2)

#right arm set point:
msg.robot_side = 1
msg.joint_angles = [0.2705597, 1.33021509, 2.099817, -0.50086, 0.02374, 0.00058,0]
msg.n_angles = len(msg.joint_angles)
lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())


print "Commanding arm"





