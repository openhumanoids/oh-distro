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
  return [roll,pitch,yaw]

def euler_to_quat(roll, pitch, yaw):
  sy = math.sin(yaw*0.5);
  cy = math.cos(yaw*0.5);
  sp = math.sin(pitch*0.5);
  cp = math.cos(pitch*0.5);
  sr = math.sin(roll*0.5);
  cr = math.cos(roll*0.5);
  w = cr*cp*cy + sr*sp*sy;
  x = sr*cp*cy - cr*sp*sy;
  y = cr*sp*cy + sr*cp*sy;
  z = cr*cp*sy - sr*sp*cy;
  return [w,x,y,z]
  
  
def on_est_robot_state(channel, data):
  m = robot_state_t.decode(data)
  pos=[]
  # = [m.pose.translation.x, m.pose.translation.y, m.pose.translation.z ]
  quat = [m.pose.rotation.w, m.pose.rotation.x, m.pose.rotation.y, m.pose.rotation.z]
  print "Quat wxyz",str(len(quat))
  qstrout = ""
  for x in quat:
    qstrout += str(x) + ", "
  print qstrout[:-2] # skip last ", "

  pos.extend( m.joint_position[0:28] )
  print "Joints",str(len(pos))
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

