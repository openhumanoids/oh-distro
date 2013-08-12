#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys
from osrf_msgs.msg import JointCommands
from sandia_hand_msgs.srv import SetJointLimitPolicy

class FingerCycleTest:
  def __init__(self):
    rospy.init_node('finger_cycle_test')
    print "waiting for services..."
    rospy.wait_for_service('set_joint_limit_policy')
    print "releasing joint limits..."
    print "done. starting cycle test..."
    self.sjlp = rospy.ServiceProxy('set_joint_limit_policy', \
                                   SetJointLimitPolicy)
    self.sjlp("none")
    self.jc_pub = rospy.Publisher('finger_0/joint_commands', JointCommands)
    self.jc = JointCommands()
    self.jc.position = [0] * 3
    self.jc.name = ["j0", "j1", "j2"]
  def run(self):
    print "running"
    rate = rospy.Rate(50.0)
    prev_time = rospy.get_time()
    a = 1.4
    targets = [ [0, 0, 0], [0, -a, -a], [0, a, a], [0, 0, 0], [-a, 0, 0], [a, 0, 0] ]
    target_idx = 0
    cycle_count = 0
    while not rospy.is_shutdown():
      rate.sleep()
      t = rospy.get_time()
      if t - prev_time > 0.75:
        self.jc.position = targets[target_idx]
        #print "sending: %s" % str(self.jc.position)
        self.jc_pub.publish(self.jc)
        prev_time = t
        target_idx = (target_idx + 1) % len(targets)
        if target_idx == 0:
          cycle_count += 1
          print "cycles: %d" % cycle_count
    print "restoring default joint limits..."
    self.sjlp("default")
if __name__ == '__main__':
  fct = FingerCycleTest()
  fct.run()

