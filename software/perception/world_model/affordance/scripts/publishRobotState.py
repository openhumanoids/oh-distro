#!/usr/bin/python
# todo: add the pelvis pitch to the yaw from the gui

import os,sys
import lcm
import time
from lcm import LCM

from threading import Thread

import drc_utils as u

home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")
from drc.walking_goal_t import walking_goal_t
from drc.atlas_behavior_manipulate_params_t import atlas_behavior_manipulate_params_t
from drc.robot_urdf_t import robot_urdf_t
from drc.robot_plan_t import robot_plan_t
from drc.atlas_status_t import atlas_status_t

from drc.atlas_behavior_feedback_t import atlas_behavior_feedback_t
from drc.atlas_stand_feedback_t import atlas_stand_feedback_t
from drc.atlas_step_feedback_t import atlas_step_feedback_t
from drc.atlas_walk_feedback_t import atlas_walk_feedback_t
from drc.atlas_manipulate_feedback_t import atlas_manipulate_feedback_t

from drc.atlas_step_data_t import atlas_step_data_t
from drc.atlas_behavior_step_spec_t import atlas_behavior_step_spec_t

from drc.atlas_behavior_foot_data_t import atlas_behavior_foot_data_t
from drc.atlas_behavior_step_action_t import atlas_behavior_step_action_t
from drc.atlas_behavior_pelvis_servo_params_t import atlas_behavior_pelvis_servo_params_t
########################################################################################

class JointNames:
    def __init__(self):
        self.left_sandia =( ["left_f0_j0","left_f0_j1","left_f0_j2",   "left_f1_j0","left_f1_j1","left_f1_j2",\
                            "left_f2_j0","left_f2_j1","left_f2_j2",   "left_f3_j0","left_f3_j1","left_f3_j2" ] )
        self.right_sandia=( ["right_f0_j0","right_f0_j1","right_f0_j2",  "right_f1_j0","right_f1_j1","right_f1_j2", \
                            "right_f2_j0","right_f2_j1","right_f2_j2",  "right_f3_j0","right_f3_j1","right_f3_j2" ] )
        self.left_irobot =( ["left_finger[0]/joint_base_rotation", "left_finger[0]/joint_base", "left_finger[0]/joint_flex", \
                            "left_finger[1]/joint_base_rotation", "left_finger[1]/joint_base", "left_finger[1]/joint_flex", \
                            "left_finger[2]/joint_base", "left_finger[2]/joint_flex" ] )
        self.right_irobot=( ["right_finger[0]/joint_base_rotation", "right_finger[0]/joint_base", "right_finger[0]/joint_flex",\
                            "right_finger[1]/joint_base_rotation", "right_finger[1]/joint_base", "right_finger[1]/joint_flex",\
                            "right_finger[2]/joint_base", "right_finger[2]/joint_flex" ] )
        self.head = ( ["pre_spindle_cal_x_joint", "pre_spindle_cal_y_joint", "pre_spindle_cal_z_joint", \
                            "pre_spindle_cal_roll_joint", "pre_spindle_cal_pitch_joint", "pre_spindle_cal_yaw_joint", \
                            "hokuyo_joint", \
                            "post_spindle_cal_x_joint", "post_spindle_cal_y_joint", "post_spindle_cal_z_joint", \
                            "post_spindle_cal_roll_joint", "post_spindle_cal_pitch_joint", "post_spindle_cal_yaw_joint" ])

def appendJoints(msg,joint_names):
  msg.num_joints = msg.num_joints + len(joint_names)
  msg.joint_position.extend( [0]*len(joint_names) )
  msg.joint_velocity.extend( [0]*len(joint_names) )
  msg.joint_effort.extend( [0]*len(joint_names) )
  msg.joint_name.extend(joint_names)
  return msg

 
def sendManipStatus():
  stat = atlas_status_t()
  stat.behavior = 6;
  stat.behavior_feedback = atlas_behavior_feedback_t();
  stat.stand_feedback = atlas_stand_feedback_t();
  stat.step_feedback = atlas_step_feedback_t();
  stat.step_feedback.desired_step_saturated = atlas_step_data_t();
  stat.step_feedback.desired_step_spec_saturated = atlas_behavior_step_spec_t();

  stat.step_feedback.desired_step_spec_saturated.foot = atlas_behavior_foot_data_t();
  stat.step_feedback.desired_step_spec_saturated.action = atlas_behavior_step_action_t();

  stat.walk_feedback = atlas_walk_feedback_t();
  stat.manipulate_feedback = atlas_manipulate_feedback_t();

  stat.manipulate_feedback.clamped = atlas_behavior_pelvis_servo_params_t()
  stat.manipulate_feedback.internal_desired = atlas_behavior_pelvis_servo_params_t()
  stat.manipulate_feedback.internal_sensed = atlas_behavior_pelvis_servo_params_t()

  lc.publish("ATLAS_STATUS", stat.encode())

def sendRobotStateMsg():
  global goal_yaw, goal_pelvis_height, goal_pelvis_pitch, goal_pelvis_roll, goal_pos, goal_xy, goal_hand_config, jnames, \
         goal_committed_use, goal_committed
  if (goal_hand_config[0] == -1):
    print "no hand config, not publishing ERS"
    return

  if (goal_committed_use==True):
    print "publishing committed as ERS"
    msg = goal_committed # will always contain 28 joints
    msg.joint_position = list(msg.joint_position[0:28])
    msg.joint_velocity = list(msg.joint_velocity[0:28])
    msg.joint_effort = list(msg.joint_effort[0:28])
    msg.joint_name = list(msg.joint_name[0:28])
    msg.num_joints = 28
    msg.utime = timestamp_now ()
  else:
    msg = u.getRobotStateMsg()
    quat_out = u.euler_to_quat(0.0,0.0, goal_yaw)
    msg.pose.rotation.w = quat_out[0]
    msg.pose.rotation.x = quat_out[1]
    msg.pose.rotation.y = quat_out[2]
    msg.pose.rotation.z = quat_out[3]
    msg.pose.translation.x = goal_xy[0]
    msg.pose.translation.y = goal_xy[1]
    print "Pelvis Height: " , goal_pelvis_height
    if (goal_pelvis_roll > 0.1):
      print "Using Leaning Pitch and Reach"
      msg = u.setStateAtHeight65BackAndPelvisPitchedAndArmExtended(msg)
    elif (goal_pelvis_pitch > 0.1):
      print "Using Leaning Pitch"
      if (goal_pelvis_height >  0.75 ):
        msg = u.setStateAtHeight85BackAndPelvisPitched(msg)
      else:
        msg = u.setStateAtHeight65BackAndPelvisPitched(msg)
    elif (goal_pelvis_height >  0.83 ):
      msg = u.setStateAtHeight86(msg)
    elif (goal_pelvis_height >  0.775 ):
      msg = u.setStateAtHeight80(msg)
    elif (goal_pelvis_height >  0.725 ):
      msg = u.setStateAtHeight75(msg)
    elif (goal_pelvis_height >  0.68 ):
      msg = u.setStateAtHeight70(msg)
    else:
      msg = u.setStateAtHeight66(msg)

  # Add the required joints:
  msg = appendJoints(msg, jnames.head)
  if (goal_hand_config[0] == 2):
    appendJoints(msg,jnames.left_sandia)
  elif (goal_hand_config[0] == 4):
    appendJoints(msg,jnames.left_irobot)
  if (goal_hand_config[1] == 3):
    msg = appendJoints(msg,jnames.right_sandia)
  elif (goal_hand_config[1] == 5):
    msg = appendJoints(msg,jnames.right_irobot)

  lc.publish("EST_ROBOT_STATE", msg.encode())
  sendManipStatus()

def on_manip_params(channel, data):
  global goal_pelvis_height,goal_pelvis_pitch,goal_pelvis_roll, goal_xy, goal_committed_use
  m = atlas_behavior_manipulate_params_t.decode(data)
  goal_pelvis_height = m.desired.pelvis_height
  goal_pelvis_pitch = m.desired.pelvis_pitch
  goal_pelvis_roll = m.desired.pelvis_roll # use roll to signify arm reach
  print "Changing height and pitch"
  goal_committed_use = False
  sendRobotStateMsg()
  
def on_walking_goal(channel, data):
  global goal_pelvis_height, goal_yaw, goal_xy, goal_committed_use
  m = walking_goal_t.decode(data)
  quat_in= [m.goal_pos.rotation.w, m.goal_pos.rotation.x, m.goal_pos.rotation.y, m.goal_pos.rotation.z]
  [roll,pitch,goal_yaw]=quat_to_euler(quat_in)
  goal_xy = [m.goal_pos.translation.x, m.goal_pos.translation.y]
  print "New walking goal"
  goal_committed_use = False
  sendRobotStateMsg()

  
def on_robot_model(channel, data):
  global goal_hand_config
  m = robot_urdf_t.decode(data)
  goal_hand_config[0] =  m.left_hand
  goal_hand_config[1] =  m.right_hand
  #print "got hands",str(goal_hand_config)


def on_committed_robot_plan(channel, data):
  global goal_committed_use, goal_committed
  m = robot_plan_t.decode(data)  
  print  "num of states ",m.num_states
  msg = m.plan[m.num_states-1]
  lc.publish("EST_ROBOT_STATE", msg.encode())
  goal_committed = msg
  goal_committed_use = True
#################################################################################

print 'drc-fake-robot-state'
#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)


lc = lcm.LCM()
print "started"

goal_xy = [0,0]
goal_yaw = 0.0
goal_pelvis_height = 0
goal_pelvis_pitch = 0
goal_pelvis_roll = 0
goal_hand_config = [-1]*2

goal_committed = []
goal_committed_use = False
jnames = JointNames()

# needed due to viewer NaN bug:
time.sleep(3)


def lcm_thread():
  sub0 = lc.subscribe("ROBOT_MODEL", on_robot_model) 
  sub1 = lc.subscribe("ATLAS_MANIPULATE_PARAMS", on_manip_params) 
  sub2 = lc.subscribe("WALKING_GOAL", on_walking_goal) 
  sub3 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_committed_robot_plan) 
  while True:
    ## Handle LCM if new messages have arrived.
    lc.handle()

  lc.unsubscribe(sub0)
  lc.unsubscribe(sub1)
  lc.unsubscribe(sub2)
  lc.unsubscribe(sub3)

t2 = Thread(target=lcm_thread)
t2.start()

sleep_timing=0.1 # time between updates of the plots - in wall time
while (1==1):
  time.sleep(sleep_timing)
  sendRobotStateMsg()

