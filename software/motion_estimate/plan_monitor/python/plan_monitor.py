#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy as np

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")
from drc.robot_plan_t import robot_plan_t
from drc.system_status_t import system_status_t
from drc.atlas_state_t import atlas_state_t
from drc.plan_control_t import plan_control_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

class State:
  def __init__(self):
    self.last_ers_utime =0
    self.got_pose_bdi =False
    self.got_pose_body =False
    self.lastRawPlan = None
    self.lastPlan = None
    self.activePlanSample = -1
    
    # 0.1 lax 
    # 0.05 too tight
    self.positionErrorThreshold = 20.10 

    self.activeFault = False

  def on_atlas_state(self,channel, data):
    m = atlas_state_t.decode(data)
    self.last_ers_utime = m.utime
    
    if (self.activePlanSample >= 0):
      unsafe_joint = self.analyzeProgress(m)
    
      if (unsafe_joint):
        self.activeFault = True
        
        msg = plan_control_t()
        msg.utime = timestamp_now()
        msg.control = 0 # pause, supposedly
        lc.publish("COMMITTED_PLAN_PAUSE", msg.encode())
        print "HALTING CONTROLLER EXECUTION - COMMITTED_PLAN_PAUSE"
        
        # flush the remaining plan:
        self.activePlanSample = -1 #
        self.lastPlan = None
        self.lastRawPlan = None
        
        
    # Clear fault
    #if ( )
   
  def analyzeProgress(self,ers):
    next_utime = self.lastPlan.plan[self.activePlanSample].utime
    
    diff_time = (ers.utime - next_utime)*1E-6
    if ( diff_time > 0.0):
      #print self.activePlanSample , " " , next_utime , " ", ers.utime , " ", diff_time
      #print "i got ", self.activePlanSample
      
      l_arm_idx = [16,17,18,19,20,21]
      all_position_plan = self.lastPlan.plan[self.activePlanSample].joint_position 
      
      l_arm_position_plan = np.asarray( [all_position_plan[i] for i in l_arm_idx] )
      l_arm_position_ers = np.asarray( [ers.joint_position[i] for i in l_arm_idx] )
      l_arm_position_error = np.absolute(l_arm_position_plan - l_arm_position_ers)
      
      unsafe = l_arm_position_error > self.positionErrorThreshold
      
      #print l_arm_position_plan
      #print l_arm_position_ers
      #print l_arm_position_error      
      #print unsafe
      
      unsafe_joint = False
      if True in unsafe:
        unsafe_joint = True
        print       
        print " @@@@@@ DANGER @@@@@@", self.activePlanSample, ers.utime, l_arm_position_error
      else:
        print self.activePlanSample
      
      # increment to the next sample
      self.activePlanSample += 1
      if (self.activePlanSample >= self.lastPlan.num_states):
        self.activePlanSample = -1 # stop
        
      return unsafe_joint
    
    
  # --- Receiving Plans ---
  def on_committed_plan(self,channel, data):
    m = robot_plan_t.decode(data)
    print "got plan, ",m.utime
    print "ers: ", self.last_ers_utime
    self.lastRawPlan = m

  def on_system_status(self,channel, data):
    m = system_status_t.decode(data)
    print "status, ",m.utime,m.value
    print "ers: \n", self.last_ers_utime
    if (self.lastRawPlan is not None):
      self.project_plan()

  def remapState(self, samp):
    samp_out = samp.decode( samp.encode() ) # decode and encode ensures deepcopy
    joint_name_out     = list(samp.joint_name)
    joint_position_out = list(samp.joint_position)
    joint_velocity_out = list(samp.joint_velocity)

    for j in range(samp.num_joints):
      j_out = self.jointMap[ samp.joint_name[j] ]
      joint_name_out[j_out] = samp.joint_name[j]
      joint_position_out[j_out] = samp.joint_position[j]
      joint_velocity_out[j_out] = samp.joint_velocity[j]
      
    samp_out.joint_name     = joint_name_out
    samp_out.joint_position = joint_position_out
    samp_out.joint_velocity = joint_velocity_out
    return samp_out
    
  def project_plan(self):
    self.lastPlan = self.lastRawPlan.decode( self.lastRawPlan.encode() ) # decode and encode ensures deepcopy
    for i in range(self.lastRawPlan.num_states):
      samp = self.remapState(self.lastRawPlan.plan[i])
      samp.utime = samp.utime + self.last_ers_utime
      self.lastPlan.plan[i] = samp
      lc.publish("EXPD_ROBOT_STATE",samp.encode())
      
    self.activePlanSample = 1
      
#################################################################################

lc = lcm.LCM()
print "started"
state = State()

jointMap = {}
jointMap['back_bkz'] = 0
jointMap['back_bky'] = 1
jointMap['back_bkx'] = 2
jointMap['neck_ay']  = 3
jointMap['l_leg_hpz'] =4
jointMap['l_leg_hpx'] =5
jointMap['l_leg_hpy'] =6
jointMap['l_leg_kny'] =7
jointMap['l_leg_aky'] =8
jointMap['l_leg_akx'] =9
jointMap['r_leg_hpz'] =10
jointMap['r_leg_hpx'] =11
jointMap['r_leg_hpy'] =12
jointMap['r_leg_kny'] =13
jointMap['r_leg_aky'] =14
jointMap['r_leg_akx'] =15
jointMap['l_arm_usy'] =16
jointMap['l_arm_shx'] =17
jointMap['l_arm_ely'] =18
jointMap['l_arm_elx'] =19
jointMap['l_arm_uwy'] =20
jointMap['l_arm_mwx'] =21
jointMap['r_arm_usy'] =22
jointMap['r_arm_shx'] =23
jointMap['r_arm_ely'] =24
jointMap['r_arm_elx'] =25
jointMap['r_arm_uwy'] =26
jointMap['r_arm_mwx'] =27
state.jointMap= jointMap

lc.subscribe("COMMITTED_ROBOT_PLAN", state.on_committed_plan)
lc.subscribe("SYSTEM_STATUS", state.on_system_status)
lc.subscribe("ATLAS_STATE", state.on_atlas_state)


while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()


