#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys
from osrf_msgs.msg import JointCommands
from sandia_hand_msgs.srv import SetJointLimitPolicy

class FingerTest:
  def __init__(self, finger_idx, j0, j1, j2):
    rospy.init_node('finger_test')
    # todo: make finger joint commands service; this is a total hack
    self.jc_pub = rospy.Publisher("finger_%d/joint_commands" % finger_idx, JointCommands)
    self.jc = JointCommands()
    self.jc.name = ["j0", "j1", "j2"]
    self.jc.position = [j0, j1, j2]
  def run(self):
    print "waiting 2 seconds for connections..."
    rate = rospy.Rate(50.0)
    prev_time = rospy.get_time()
    while not rospy.is_shutdown():
      rate.sleep()
      t = rospy.get_time()
      if t - prev_time > 2.0: # total hack. fix this abomination sometime.
        self.jc_pub.publish(self.jc)
        print "sent"
        break

if __name__ == '__main__':
  if len(rospy.myargv()) != 5:
    print "usage: finger_joint_pos.py FINGER_IDX J0 J1 J2"
    sys.exit(1)
  ft = FingerTest(int(rospy.myargv()[1]), float(rospy.myargv()[2]), float(rospy.myargv()[3]), float(rospy.myargv()[4]))
  ft.run()

