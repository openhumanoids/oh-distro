#!/usr/bin/python
import os,sys
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.robot_state_t import robot_state_t
from bot_core.pose_t import pose_t
import time
import numpy as np
import math

import multisense as lcmmultisense

def timestamp_now (): return int (time.time () * 1000000)

print "drc-send-robot-state [v5|val1|val2|multisense]"

if len(sys.argv) > 1:
  robot_name = sys.argv[1]
else:
  robot_name = 'val2'

if len(sys.argv) > 2:
  mode = sys.argv[2]
else:
  mode = 'static'

if len(sys.argv) > 3:
  rotation_flag = sys.argv[3]
else:
  rotation_flag = 'no-rotate'  

print robot_name


def euler_to_quat(rpy):
  roll =  rpy[0]
  pitch = rpy[1]
  yaw =   rpy[2]

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
  return np.array([w,x,y,z])


if robot_name == "v5":
  print "Atlas Version 5"
  joint_names = ['back_bkz', 'back_bky', 'back_bkx', 'neck_ay',  
                 'r_arm_shz','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_uwy','r_arm_mwx','r_arm_lwy', 
                 'l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwy',
                 'r_leg_hpz','r_leg_hpx','r_leg_hpy','r_leg_kny','r_leg_aky','r_leg_akx',
                 'l_leg_hpz','l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx']
elif robot_name == "val1":
  print "Valkyrie Version 1"
  joint_names = ["WaistRotator",    "WaistExtensor",    "WaistLateralExtensor",    "LowerNeckExtensor",
    "NeckRotator",    "UpperNeckExtensor",    "RightShoulderExtensor",    "RightShoulderAdductor",
    "RightShoulderSupinator",    "RightElbowExtensor",    "RightForearmSupinator",    "RightWristExtensor",
    "RightWrist",    "LeftShoulderExtensor",    "LeftShoulderAdductor",    "LeftShoulderSupinator",
    "LeftElbowExtensor",    "LeftForearmSupinator",    "LeftWristExtensor",    "LeftWrist",    "LeftHipRotator",    
    "LeftHipAdductor",    "LeftHipExtensor",    "LeftKneeExtensor",    "LeftAnkleExtensor",    "LeftAnkle",    
    "RightHipRotator",   "RightHipAdductor",    "RightHipExtensor",    "RightKneeExtensor",    "RightAnkleExtensor",    "RightAnkle"]
elif robot_name == "val2":
  print "Valkyrie Version 2"
  joint_names = ["torsoYaw",    "torsoPitch",    "torsoRoll",    "lowerNeckPitch",    "neckYaw",    "upperNeckPitch",
    "rightShoulderPitch",     "rightShoulderRoll",    "rightShoulderYaw",    "rightElbowPitch",    "rightForearmYaw",    "rightWristRoll",
    "rightWristPitch",    "leftShoulderPitch",    "leftShoulderRoll",    "leftShoulderYaw",    "leftElbowPitch",    "leftForearmYaw",
    "leftWristRoll",    "leftWristPitch",    "leftHipYaw",    "leftHipRoll",    "leftHipPitch",    "leftKneePitch",    "leftAnklePitch",
    "leftAnkleRoll",    "rightHipYaw",    "rightHipRoll",    "rightHipPitch",    "rightKneePitch",    "rightAnklePitch",    "rightAnkleRoll"]
elif robot_name == "multisense":
  print "Multisense"
  joint_names = ["hokuyo_joint"]

lc = lcm.LCM()
print "Sending EST_ROBOT_STATE for ", robot_name
msg = robot_state_t();
msg.utime = 0;
msg.joint_name = joint_names
msg.num_joints =len(msg.joint_name);
msg.joint_position = [0]*msg.num_joints;
msg.joint_velocity = [0]*msg.num_joints;
msg.joint_effort = [0]*msg.num_joints;

msg2 = pose_t()
msg2.utime = msg.utime


if mode == "static":
  print "Send a single pose..."
  msg.pose.translation.x = 0
  msg.pose.translation.y = 0
  msg.pose.translation.z = 1.2
  msg.joint_position[0] = 0
  orientation = euler_to_quat([0, 0.0*np.pi/180.0,0])
  msg.pose.rotation.w = orientation[0]
  msg.pose.rotation.x = orientation[1]
  msg.pose.rotation.y = orientation[2]
  msg.pose.rotation.z = orientation[3]
  lc.publish("EST_ROBOT_STATE", msg.encode())

  msg2.pos = [msg.pose.translation.x, msg.pose.translation.y, msg.pose.translation.z]
  msg2.orientation = [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z]
  lc.publish("POSE_BODY", msg2.encode())
  
  if rotation_flag == 'rotate':
    time.sleep(5)
    msg3 = lcmmultisense.command_t()
    msg3.utime = getUtime()
    msg3.fps = 15
    msg3.gain = -1
    msg3.exposure_us = 10000
    msg3.agc = -1
    msg3.rpm = 5
    msg3.leds_flash = False
    msg3.leds_duty_cycle = 0
    lc.publish("MULTISENSE_COMMAND", msg3.encode())
    print "Publishing Multisense command to spin at 5rpm"

else:
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

    msg2.pos = [msg.pose.translation.x, msg.pose.translation.y, msg.pose.translation.z]
    msg2.orientation = [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z]
    lc.publish("POSE_BODY", msg2.encode())

    time.sleep(0.01)
