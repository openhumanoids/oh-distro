#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.robot_plan_t import robot_plan_t
from valkyrie.hand_pose_packet_message_t import hand_pose_packet_message_t

def timestamp_now (): return int (time.time () * 1000000)


def on_bw(channel, data):
  m = robot_plan_t.decode(data)
  estate =  m.plan[m.num_states-1]

  estate_time = estate.utime*1E-6
  print estate_time

  l_arm_names = ['l_arm_shz','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx']
  l_arm_angles = []
  for i in range(0,6):
    ix= estate.joint_name.index(l_arm_names[i])
    val = estate.joint_position[ix]
    l_arm_angles.extend([val])

  #print l_arm_angles

  msg = hand_pose_packet_message_t()
  msg.utime = m.utime
  msg.reference_frame = 0 # not used with joints
  msg.position = [0,0,0] # ee position and orientation
  msg.orientation = [1,0,0,0]
  msg.robot_side = 0
  msg.trajectory_time = estate_time

  print 'No mode specified - sending null config'
  msg.data_type =  1 # ee pos=0 | joint angles =1
  msg.to_home_position = False
  msg.n_angles = len(l_arm_angles)
  msg.joint_angles = l_arm_angles

  lc.publish("VAL_COMMAND_HAND_POSE", msg.encode())
  print "Sending through plan"



lc = lcm.LCM()
sub1 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_bw) # required
while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
