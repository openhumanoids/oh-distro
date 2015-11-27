#!/usr/bin/python
import os,sys
import lcm
import time
import re

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.robot_plan_t import robot_plan_t
from ipab.hand_pose_packet_message_t import hand_pose_packet_message_t

def timestamp_now (): return int (time.time () * 1000000)

def on_plan(channel, data):
  global atlas_version
  m = robot_plan_t.decode(data)
  estate =  m.plan[m.num_states-1]

  estate_time = estate.utime*1E-6
  print estate_time

  if ('LeftShoulderExtensor' in estate.joint_name):
    print('received plan for valkyrie')
    l_arm_names = ['LeftShoulderExtensor','LeftShoulderAdductor','LeftShoulderSupinator','LeftElbowExtensor','LeftForearmSupinator','LeftWristExtensor','LeftWrist']
    r_arm_names = ['RightShoulderExtensor','RightShoulderAdductor','RightShoulderSupinator','RightElbowExtensor','RightForearmSupinator','RightWristExtensor','RightWrist']
  else:
    regex = re.compile('l_arm*')
    l_arm_names = [string for string in estate.joint_name if re.match(regex,string)]
    regex = re.compile('r_arm*')
    r_arm_names = [string for string in estate.joint_name if re.match(regex,string)]



  print 'sending' , len(l_arm_names) , 'left and' , len(r_arm_names) , 'right joints'
  l_arm_angles = []
  for i in range(0,len(l_arm_names)):
    ix= estate.joint_name.index(l_arm_names[i])
    val = estate.joint_position[ix]
    l_arm_angles.extend([val])

  r_arm_angles = []
  for i in range(0,len(r_arm_names)):
    ix= estate.joint_name.index(r_arm_names[i])
    val = estate.joint_position[ix]
    r_arm_angles.extend([val])



  #print l_arm_angles

  msg = hand_pose_packet_message_t()
  msg.utime = m.utime
  msg.reference_frame = 0 # not used with joints
  msg.position = [0,0,0] # ee position and orientation
  msg.orientation = [1,0,0,0]
  msg.trajectory_time = estate_time
  msg.data_type =  1 # ee pos=0 | joint angles =1

  msg.robot_side = 0
  msg.to_home_position = False
  msg.n_angles = len(l_arm_angles)
  msg.joint_angles = l_arm_angles
  lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
  print "Sending through left1"
  time.sleep(1)

  msg.robot_side = 1
  msg.to_home_position = False
  msg.n_angles = len(r_arm_angles)
  msg.joint_angles = r_arm_angles
  lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
  print "Sending through plan right1"

  msg.robot_side = 0
  msg.to_home_position = False
  msg.n_angles = len(l_arm_angles)
  msg.joint_angles = l_arm_angles
  lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
  print "Sending through left2"
  time.sleep(1)

  msg.robot_side = 1
  msg.to_home_position = False
  msg.n_angles = len(r_arm_angles)
  msg.joint_angles = r_arm_angles
  lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
  print "Sending through plan right2"



lc = lcm.LCM()
sub1 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_plan) # required
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
