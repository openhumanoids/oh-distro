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
from drc.robot_state_t import robot_state_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)


def quat_to_euler(q) :
  
  roll_a = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  roll_b = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  roll = math.atan2 (roll_a, roll_b);

  pitch_sin = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  pitch = math.asin (pitch_sin);

  yaw_a = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  yaw_b = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);  
  yaw = math.atan2 (yaw_a, yaw_b);
  
  #a = q[0]
  #b = q[1]
  #c = q[2]  
  #d = q[3]  
  #roll = math.atan2(2.0*(c*d + b*a),a*a-b*b-c*c+d*d);
  #pitch = math.asin(-2.0*(b*d - a*c));
  #yaw = math.atan2(2.0*(b*c + d*a),a*a+b*b-c*c-d*d);

  
  
  
  #roll = math.atan2( 2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  #pitch = math.asin(2*(q[0]*q[2]-q[3]*q[1]));
  #yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
  return [roll,pitch,yaw]

  
  
  
def on_est_robot_state(channel, data):
  m = robot_state_t.decode(data)
  pos = [m.pose.translation.x, m.pose.translation.y, m.pose.translation.z ]
  rpy = quat_to_euler([m.pose.rotation.w, m.pose.rotation.x, m.pose.rotation.y, m.pose.rotation.z] )
  scale = 180.0/math.pi
  pos.extend(rpy)
  pos.extend( m.joint_position[0:28] )
  #print pos
  print len(pos)
  strout = ""
  for x in pos:
    strout += str(x) + ", "
  print strout[:-2] # skip last ", "
  

#################################################################################

lc = lcm.LCM()
print "started"

lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
  
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

