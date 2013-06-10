#!/usr/bin/python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
import csv_parse 

from atlas_msgs.msg import AtlasCommand
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil

currentJointState = JointState()
def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg



if __name__ == '__main__':
  if len(sys.argv) < 2:
    print "usage: traj_yaml.py crawling_gains.csv"
    print "  where crawling_gains.csv is the gains and positions file"
    sys.exit(1)

    

  # Setup subscriber to atlas states
  rospy.Subscriber("/atlas/joint_states", JointState, jointStatesCallback)

  d=csv_parse.csv_parse(sys.argv[1])

  direction =1
  if len(sys.argv) == 2:
    traj_steps = range(0, d.traj_len)
    print "doing all %d steps" %( len(traj_steps) ) 
  elif len(sys.argv) == 3:
    traj_steps = int(sys.argv[2]) 
    if (traj_steps == -1):
      direction = -1 # reverses the sequence.
      traj_steps = reversed(range(0,d.traj_len))
    else:
      traj_steps =xrange(traj_steps-1,traj_steps)
      print "just using step %s" % (sys.argv[2]) 

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
      rospy.sleep(dt / float(n))
