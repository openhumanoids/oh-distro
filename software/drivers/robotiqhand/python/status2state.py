#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.hand_state_t import hand_state_t
from robotiqhand.status_t import status_t
########################################################################################


l_names = [ "left_finger_1_joint_1", "left_finger_1_joint_2", "left_finger_1_joint_3",
        "left_finger_2_joint_1", "left_finger_2_joint_2", "left_finger_2_joint_3",
        "left_finger_middle_joint_1", "left_finger_middle_joint_2", "left_finger_middle_joint_3",
        "left_palm_finger_1_joint", "left_palm_finger_2_joint"]
r_names = [ "right_finger_1_joint_1", "right_finger_1_joint_2", "right_finger_1_joint_3",
        "right_finger_2_joint_1", "right_finger_2_joint_2", "right_finger_2_joint_3",
        "right_finger_middle_joint_1", "right_finger_middle_joint_2", "right_finger_middle_joint_3",
        "right_palm_finger_1_joint", "right_palm_finger_2_joint"]

def timestamp_now (): return int (time.time () * 1000000)

def get_mapping(input_val):
  return 1.0*input_val/255

def on_status(channel, data):
  m = status_t.decode(data)

  o = hand_state_t()
  o.utime = m.utime
  o.num_joints = 11

  o.joint_velocity = [0]*11
  o.joint_effort = [0]*11
  o.joint_position = [0]*11

  o.joint_position[0] = get_mapping(m.positionA)
  o.joint_position[1] = get_mapping(m.positionA)
  o.joint_position[2] = get_mapping(m.positionA)

  o.joint_position[3] = get_mapping(m.positionB)
  o.joint_position[4] = get_mapping(m.positionB)
  o.joint_position[5] = get_mapping(m.positionB)

  o.joint_position[6] = get_mapping(m.positionC)
  o.joint_position[7] = get_mapping(m.positionC)
  o.joint_position[8] = get_mapping(m.positionC)

  o.joint_position[9] = -0.002 * (m.positionS-137)
  o.joint_position[10] = 0.002 * (m.positionS-137)

  if (channel == "ROBOTIQ_RIGHT_STATUS"):
    o.joint_name= r_names
    lc.publish("ROBOTIQ_RIGHT_STATE", o.encode())
  else:
    o.joint_name= l_names
    lc.publish("ROBOTIQ_LEFT_STATE", o.encode())
 

#################################################################################

lc = lcm.LCM()
print "status to state"

lc.subscribe("ROBOTIQ_RIGHT_STATUS", on_status)
lc.subscribe("ROBOTIQ_LEFT_STATUS", on_status)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

