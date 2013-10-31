#!/usr/bin/python
# todo: add the pelvis pitch to the yaw from the gui

import os,sys
import lcm
import time
from lcm import LCM
import math
from threading import Thread

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")
from drc.hand_state_t import hand_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

def sendRobotStateMsg():
  msg = hand_state_t()
  msg.utime = timestamp_now ()

  global hand_side, hand_type
  
  #msg = appendSandiaJoints(msg)
  if ((hand_side == "left") and (hand_type == "sandia")):
    msg.joint_name.extend( ["left_f0_j0","left_f0_j1","left_f0_j2",   "left_f1_j0","left_f1_j1","left_f1_j2",\
      "left_f2_j0","left_f2_j1","left_f2_j2",   "left_f3_j0","left_f3_j1","left_f3_j2" ] )
    channel = "SANDIA_LEFT_STATE"
  elif ((hand_side == "right") and (hand_type == "sandia")):
    msg.joint_name.extend( ["right_f0_j0","right_f0_j1","right_f0_j2",  "right_f1_j0","right_f1_j1","right_f1_j2", \
      "right_f2_j0","right_f2_j1","right_f2_j2",  "right_f3_j0","right_f3_j1","right_f3_j2" ] )
    channel = "SANDIA_RIGHT_STATE"
  elif ((hand_side == "left") and (hand_type == "irobot")):
    msg.joint_name.extend( ["left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base", "left_finger[0]/joint_flex", \
      "left_finger[1]/joint_base_rotation", "left_finger[1]/joint_base", "left_finger[1]/joint_flex", \
      "left_finger[2]/joint_base", "left_finger[2]/joint_flex" ] )
    channel = "IROBOT_LEFT_STATE"
  elif ((hand_side == "right") and (hand_type == "irobot")):
    msg.joint_name.extend( ["right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base", "right_finger[0]/joint_flex",\
        "right_finger[1]/joint_base_rotation", "right_finger[1]/joint_base", "right_finger[1]/joint_flex",\
        "right_finger[2]/joint_base", "right_finger[2]/joint_flex" ] )
    channel = "IROBOT_RIGHT_STATE"
  else:
    print "mode not recognised"
    return
  
  #print msg.joint_name
  #print len(msg.joint_name)


  msg.num_joints = len(msg.joint_name)
  msg.joint_position.extend ( [0]*len(msg.joint_name) )
  msg.joint_velocity.extend ( [0]*len(msg.joint_name) )
  msg.joint_effort.extend ( [0]*len(msg.joint_name) )
  print "Sending", channel
  lc.publish(channel, msg.encode())

#################################################################################



print 'drc-fake-hand-state [left|right] [irobot|sandia]'
#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)

hand_side = "left"
hand_type = "sandia"
if (len(sys.argv)>=2):
  hand_side = sys.argv[1]
  print "hand_side:", hand_side

else:
  print "hand_side defaulting to", hand_side

if (len(sys.argv)>=3):
  hand_type = sys.argv[2]
  print "hand_type:", hand_type
else:
  print "hand_type defaulting to ", hand_type


lc = lcm.LCM()


sleep_timing=0.5 # time between updates
while (1==1):
  time.sleep(sleep_timing)
  sendRobotStateMsg()

