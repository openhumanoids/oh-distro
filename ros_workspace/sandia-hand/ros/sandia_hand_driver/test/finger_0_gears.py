#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys
from sandia_hand_msgs.srv import SetParameters, SetParametersRequest
from sandia_hand_msgs.msg import Parameter

class SetFingerGears:
  def __init__(self):
    rospy.init_node('set_finger_gears')
    if len(rospy.myargv()) != 4:
      print "usage: finger_0_gears.py GEAR_0 GEAR_1 GEAR_2"
      sys.exit(1)
    print "waiting for services..."
    rospy.wait_for_service('finger_0/set_parameters')
    self.sp = rospy.ServiceProxy('finger_0/set_parameters', SetParameters)
    print "done"
  def run(self):
    print "running"
    gears = [float(rospy.myargv()[1]), float(rospy.myargv()[2]), float(rospy.myargv()[3])]
    spr = SetParametersRequest()
    for i in xrange(0,3):
      spr.parameters.append(Parameter())
      spr.parameters[i].val_type = Parameter.FLOAT
      spr.parameters[i].name  = "m%d_gear" % i
      spr.parameters[i].f_val = float(rospy.myargv()[i+1])
    self.sp(spr)

if __name__ == '__main__':
  sfg = SetFingerGears()
  sfg.run()

