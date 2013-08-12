#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys, select, yaml
from sandia_hand_msgs.msg import RawFingerStatus

class AccelLogger:
  def __init__(self, argv):
    rospy.init_node('accel_logger')
    self.finger_sub = rospy.Subscriber('raw_status', RawFingerStatus, 
                                       self.status_cb)
    print "press [enter] to record accel readings"
    self.last_raw_msg = None
    self.history = []

  def status_cb(self, msg):
    self.last_raw_msg = msg

  def record_accels(self):
    if self.last_raw_msg == None:
      print "nothing to record"
      return
    m = self.last_raw_msg # save typing
    y = { "mm": list(m.mm_accel), 
          "pp": list(m.dp_accel), 
          "dp": list(m.pp_accel) }
    self.history.append(y)
    yaml.dump(self.history, file('accel_log.yaml', 'w'))
    print yaml.dump(self.history)
    print "recording accels"

  def spin(self):
    while not rospy.is_shutdown():
      try:
        i,o,e = select.select([sys.stdin],[],[],0.01)
      except select.error:
        pass
      for s in i:
        if s == sys.stdin:
          s.readline()
          self.record_accels()
      #rospy.sleep(0.01)

if __name__ == '__main__':
  try:
    al = AccelLogger(rospy.myargv())
    al.spin()
  except rospy.ROSInterruptException:
    pass
