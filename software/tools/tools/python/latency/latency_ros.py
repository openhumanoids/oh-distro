#!/usr/bin/env python
# given a specific joint_state message
import roslib; roslib.load_manifest('scope_latency')
import rospy
from sensor_msgs.msg import JointState
from osrf_msgs.msg import JointCommands

from drc_latency import Latency

def js_callback(data):
  js_time = int (data.header.stamp.secs*1E9 + data.header.stamp.nsecs)
  #print js_time
  s.add_from(js_time, s.timestamp_now_sec())

def jc_callback(data):
  jc_time = int (data.header.stamp.secs*1E9 + data.header.stamp.nsecs)
  s.add_to(jc_time, s.timestamp_now_sec(),"ROS")

def listener():
  rospy.init_node('scope_latency', anonymous=True)
  rospy.Subscriber("atlas/joint_states", JointState, js_callback)
  rospy.Subscriber("atlas/joint_commands", JointCommands, jc_callback)
  rospy.spin()

s = Latency(0, 0)

if __name__ == '__main__':
  listener()
