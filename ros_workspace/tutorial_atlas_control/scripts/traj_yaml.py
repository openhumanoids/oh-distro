#!/usr/bin/python
import roslib; roslib.load_manifest('tutorial_atlas_control')
import rospy, yaml, sys
import csv_parse 

from osrf_msgs.msg import JointCommands
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

  if len(sys.argv) == 2:
    traj_steps = xrange(0, d.traj_len)
    print "doing all %d steps" %( len(traj_steps) ) 
  elif len(sys.argv) == 3:
    traj_steps = int(sys.argv[2]) 
    traj_steps =xrange(traj_steps-1,traj_steps)
    print "just using step %s" % (sys.argv[2]) 

  # initialize JointCommands message
  command = JointCommands()
  n = len(d.joints)
  command.position     = zeros(n)
  command.velocity     = zeros(n)
  command.effort       = zeros(n)
  command.kp_position  = zeros(n)
  command.ki_position  = zeros(n)
  command.kd_position  = zeros(n)
  command.kp_velocity  = zeros(n)
  command.i_effort_min = zeros(n)
  command.i_effort_max = zeros(n)

  # now get gains from parameter server
  rospy.init_node('tutorial_atlas_control')
  for i in xrange(len(d.joints)):
    name = d.joints[i].name
    command.name.append(d.joints[i].name)
    command.kp_position[i]  = rospy.get_param('atlas_controller/gains/' + name + '/p')
    command.ki_position[i]  = rospy.get_param('atlas_controller/gains/' + name + '/i')
    command.kd_position[i]  = rospy.get_param('atlas_controller/gains/' + name + '/d')
    command.i_effort_max[i] = rospy.get_param('atlas_controller/gains/' + name + '/i_clamp')
    command.i_effort_min[i] = -command.i_effort_max[i]

  # set up the publisher
  pub = rospy.Publisher('/atlas/joint_commands', JointCommands)

  # for each trajectory
  print traj_steps
  for i in traj_steps:
    print i
    # get initial joint positions
    initialPosition = array(currentJointState.position)
    # time duration
    dt =1

    # desired joint positions
    commandPosition2 =[]
    for j in xrange(len(d.joints)):
      commandPosition2.append( d.joints[j].points[i] )
    commandPosition = array(commandPosition2)

    # desired publish interval
    dtPublish = 0.02
    n = ceil(dt / dtPublish)
    for ratio in linspace(0, 1, n):
      interpCommand = (1-ratio)*initialPosition + ratio * commandPosition
      command.position = [ float(x) for x in interpCommand ]
      pub.publish(command)
      rospy.sleep(dt / float(n))
