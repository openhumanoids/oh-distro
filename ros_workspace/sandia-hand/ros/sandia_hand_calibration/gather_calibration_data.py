#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_calibration')
import rospy, sys, math, random
from std_msgs.msg import Float64

class FingerDataCollector:
  def __init__(self, argv):
    rospy.init_node('finger_data_collector')
    self.tilt_pub = rospy.Publisher('tilt_controller/command', Float64)
    self.roll_pub = rospy.Publisher('roll_controller/command', Float64)
  def go(self):
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
      tilt = -3.14159 * (random.random())
      roll =  3.14159 * (random.random() - 0.5)
      print "(%.3f, %.3f)" % (tilt, roll)
      self.roll_pub.publish(Float64(roll))
      self.tilt_pub.publish(Float64(tilt))
      rate.sleep()

if __name__ == '__main__':
  fdc = FingerDataCollector(rospy.myargv())
  fdc.go()
