#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from valkyrie.hand_pose_packet_message_t import hand_pose_packet_message_t

def timestamp_now (): return int (time.time () * 1000000)

msg = hand_pose_packet_message_t()
msg.utime = timestamp_now()
msg.robot_side = 0
msg.data_type =  1
msg.reference_frame = 0 # not used with joints
msg.to_home_position = False
msg.position = [0,0,0]
msg.orientation = [1,0,0,0]
msg.trajectory_time = 10
msg.n_angles = 6;
msg.joint_angles = [0,0,0,0,0,0]

lc = lcm.LCM()
lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
print "Commanding arm"

