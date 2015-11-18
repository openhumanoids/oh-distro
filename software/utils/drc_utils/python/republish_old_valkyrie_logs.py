#!/usr/bin/python
import os,sys
import time

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

import lcm
from drc.robot_state_t import robot_state_t
import time

lc = lcm.LCM()


mappings = {
  'l_leg_hpz':'LeftHipRotator',
  'l_leg_hpx':'LeftHipAdductor',
  'l_leg_hpy':'LeftHipExtensor',
  'l_leg_kny':'LeftKneeExtensor',
  'l_leg_aky':'LeftAnkleExtensor',
  'l_leg_akx':'LeftAnkle',
  'r_leg_hpz':'RightHipRotator',
  'r_leg_hpx':'RightHipAdductor',
  'r_leg_hpy':'RightHipExtensor',
  'r_leg_kny':'RightKneeExtensor',
  'r_leg_aky':'RightAnkleExtensor',
  'r_leg_akx':'RightAnkle',
  'back_bkz':'WaistRotator',
  'back_bky':'WaistExtensor',
  'back_bkx':'WaistLateralExtensor',
  'l_arm_shz':'LeftShoulderExtensor',
  'l_arm_shx':'LeftShoulderAdductor',
  'l_arm_ely':'LeftShoulderSupinator',
  'l_arm_elx':'LeftElbowExtensor',
  'l_arm_uwy':'LeftForearmSupinator',
  'l_arm_mwx':'LeftWristExtensor',
  'l_arm_lwy':'LeftWrist',
  'r_arm_shz':'RightShoulderExtensor',
  'r_arm_shx':'RightShoulderAdductor',
  'r_arm_ely':'RightShoulderSupinator',
  'r_arm_elx':'RightElbowExtensor',
  'r_arm_uwy':'RightForearmSupinator',
  'r_arm_mwx':'RightWristExtensor',
  'r_arm_lwy':'RightWrist',
  'back_bkz':'WaistRotator',
  'back_bky':'WaistExtensor',
  'back_bkx':'WaistLateralExtensor',
  'neck_ay':'LowerNeckExtensor',
  'neck_by':'NeckRotator',
  'neck_cy':'UpperNeckExtensor'}




def on_msg1(channel, data):
  m = robot_state_t.decode(data)

  for i in range(len(m.joint_name)):
    joint = m.joint_name[i]
    joint_out = mappings.get(joint)
    if (joint_out is not None):
      m.joint_name[i] = joint_out
    else:
      print joint , " not found"
    
  lc.publish('EST_ROBOT_STATE', m.encode() )


sub1 = lc.subscribe("EST_ROBOT_STATE_ORIGINAL", on_msg1)

while True:
  lc.handle()
