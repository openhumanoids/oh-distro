#!/usr/bin/python
import roslib; roslib.load_manifest('multisense_command')
import rospy

from multisense_msgs.msg import *
from actionlib_msgs.msg import *

print 'multisense_command.py [speed|led|flash] [float 0-1]'
print 'Control sensor head'
print 'Argument List:', str(sys.argv)
if len(sys.argv)==3:
  mode = sys.argv[1]
  command = sys.argv[2]
  command_f = float(command)
else:
  print 'No mode or command specified - exiting'
  sys.exit(0)

def led(command):
  pub2 = rospy.Publisher('/led_control/goal', LedControlActionGoal)
  rospy.init_node('multisense_control')
  msg = LedControlActionGoal()
  msg.goal.flashing =False
  msg.goal.intensities= [ command , command, command, command]
  print "sent led on/off control"
  for i in range(4):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id =''
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = "/led_control-1-"+str(msg.header.stamp.secs ) +"."+ str(msg.header.stamp.nsecs/ 1000000 )
    pub2.publish(msg)
    rospy.sleep(0.2)

def flash(command):
  pub2 = rospy.Publisher('/led_control/goal', LedControlActionGoal)
  rospy.init_node('multisense_control')
  msg = LedControlActionGoal()
  goal = LedControlGoal()
  if ( command > 0.5):
    msg.goal.flashing =True
  else:
    msg.goal.flashing =False
  
  msg.goal.intensities= [ 0.1, 0.1, 0.1, 0.1]
  print "sent flash control"
  for i in range(4):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id =''
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = "/led_control-1-"+str(msg.header.stamp.secs ) +"."+ str(msg.header.stamp.nsecs/ 1000000 )
    pub2.publish(msg)
    rospy.sleep(0.2)

def spindle_speed(command):
  pub2 = rospy.Publisher('/laser_joint/spindle_control/goal', SpindleControlActionGoal)
  rospy.init_node('multisense_control')
  msg = SpindleControlActionGoal()
  msg.goal.rotational_speed = command
  print "sent spindle speed control"
  for i in range(2):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id =''
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = "/spindle_control-1-"+str(msg.header.stamp.secs ) +"."+ str(msg.header.stamp.nsecs/ 1000000 )
    pub2.publish(msg)
    rospy.sleep(0.2)

if (mode=='speed'): 
  spindle_speed(command_f)
elif(mode=='led'): 
  led(command_f)
elif(mode=='flash'): 
  flash(command_f)
else:
  print "mode must be either speed, ledon or ledoff - not",mode
  sys.exit(0)
