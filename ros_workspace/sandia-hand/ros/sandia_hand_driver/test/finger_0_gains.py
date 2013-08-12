#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys
from sandia_hand_msgs.srv import SetParameters, SetParametersRequest
from sandia_hand_msgs.msg import Parameter

class SetFingerGains:
  def __init__(self):
    rospy.init_node('set_finger_gains')
    if len(rospy.myargv()) != 4:
      print "usage: finger_0_gains.py P_GAIN D_GAIN TORQUE_LIMIT"
      sys.exit(1)
    print "waiting for services..."
    rospy.wait_for_service('finger_0/set_parameters')
    self.sp = rospy.ServiceProxy('finger_0/set_parameters', SetParameters)
    print "done"
  def run(self):
    print "running"
    p_gain = float(rospy.myargv()[1])
    d_gain = float(rospy.myargv()[2])
    torque_limit  = int(rospy.myargv()[3])
    spr = SetParametersRequest()
    for i in xrange(0,6):
      spr.parameters.append(Parameter())
      spr.parameters[i].val_type = Parameter.FLOAT
    for i in xrange(6,9):
      spr.parameters.append(Parameter())
      spr.parameters[i].val_type = Parameter.INTEGER
    for i in xrange(0,3):
      spr.parameters[i  ].name  = "m%d_kp" % i
      spr.parameters[i+3].name  = "m%d_kd" % i
      spr.parameters[i+6].name  = "m%d_torque_limit" % i
      spr.parameters[i  ].f_val = p_gain
      spr.parameters[i+3].f_val = d_gain
      spr.parameters[i+6].i_val = torque_limit
    self.sp(spr)

if __name__ == '__main__':
  sfg = SetFingerGains()
  sfg.run()

