#!/usr/bin/env python
## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import roslib; roslib.load_manifest('mit_drcsim_scripts')

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def talker():
  pub = rospy.Publisher('/atlas/set_pose', Pose)
  rospy.init_node('talker', anonymous=True)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    str = "hello world %s"%rospy.get_time()
    rospy.loginfo(str)
    
    p = Pose()
    p.position.z = 2
    pub.publish(p)
    r.sleep()
    
if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException: pass
