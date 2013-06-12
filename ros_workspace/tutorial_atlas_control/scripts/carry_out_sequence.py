#!/usr/bin/python
# Filename: carry_out_sequence.py

import rospy, yaml, sys
import csv_parse
import time

from atlas_msgs.msg import AtlasCommand
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil

currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

def timestamp_now (): return int (time.time () * 1000000)


def carry_out_sequence(filename, which_steps, real_time_percent):
  # Setup subscriber to atlas states
  rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)


  d=csv_parse.csv_parse(filename)
  if which_steps == 0:
    traj_steps = range(0, d.traj_len)
    print "doing all %d steps" %( len(traj_steps) ) 
  else:
    traj_steps = int(which_steps) 
    if (traj_steps == -1):
      traj_steps = reversed(range(0,d.traj_len))
      print "reversing the sequence"
    else:
      traj_steps =xrange(traj_steps-1,traj_steps)
      print "just using step %s" % (which_steps) 

  # now get gains from parameter server
  rospy.init_node('tutorial_atlas_control')


  # initialize AtlasCommand message
  command = AtlasCommand()
  n = len(d.joints)
  command.position     = zeros(n)
  command.velocity     = tuple(zeros(n))
  command.effort       = tuple(zeros(n))
  command.kp_position  = tuple(zeros(n))
  command.ki_position  = tuple(zeros(n))
  command.kd_position  = tuple(zeros(n))
  command.kp_velocity  = tuple(zeros(n))
  command.k_effort  = tuple(zeros(n) +255) 
  command.i_effort_min = tuple(zeros(n))
  command.i_effort_max = tuple(zeros(n))
  command.desired_controller_period_ms = 5

  kp_position = [None]*n
  ki_position = [None]*n
  kd_position = [None]*n
  i_effort_max = [None]*n
  i_effort_min = [None]*n
  for i in xrange(len(d.joints)):
    name = d.joints[i].name
    kp_position[i]  = d.joints[i].kp
    kd_position[i]  = d.joints[i].kd
    ki_position[i]  = 0
    i_effort_max[i] = 0
    i_effort_min[i] = 0

  command.kp_position = tuple(kp_position)
  command.ki_position = tuple(ki_position)
  command.kd_position = tuple(kd_position)
  command.i_effort_max = tuple(i_effort_max)
  command.i_effort_min = tuple(i_effort_min)
  pub2 = rospy.Publisher('/atlas/atlas_command', AtlasCommand)

# 0.1 it sleeps by 100,000

# 0.1 === > 

# sleeps 0.01 simulated time
# if operating at 0.1
# sleeps 0.1 wall time =  0.01 / 0.1

  # for each trajectory
  print traj_steps
  for i in traj_steps:
    print i
    # get initial joint positions
    initialPosition = array(currentJointState.position)
    # time duration
    dt =d.dt[i]

    # desired joint positions
    commandPosition2 =[]
    for j in xrange(len(d.joints)):
      commandPosition2.append( d.joints[j].points[i] )
    commandPosition = array(commandPosition2)

    # desired publish interval
    dtPublish = 0.01
    n = ceil(dt / dtPublish)
    for ratio in linspace(0, 1, n):
      interpCommand = (1-ratio)*initialPosition + ratio * commandPosition
      command.position = [ float(x) for x in interpCommand ]
      pub2.publish(command)
      
      #ros_sleep_amount =  dt / float(n)
      #ros_tic =timestamp_now ()
      #rospy.sleep( ros_sleep_amount )
      #ros_delta = timestamp_now () - ros_tic
      #print "ros_delta %f" % (ros_delta)
      #print "ros_sleep_amount %f" % (ros_sleep_amount)

      lcm_sleep_amount = dtPublish * 100/ real_time_percent  + 0.011 
      # this is required for the alienware machines - working on removing it
      # lower might be the best number
      lcm_tic =timestamp_now ()
      time.sleep(lcm_sleep_amount)
      lcm_delta = timestamp_now () - lcm_tic
      #print "lcm_delta %f [%d]" % (lcm_delta, real_time_percent)
      #print "lcm_sleep_amount %f" % (lcm_sleep_amount)



