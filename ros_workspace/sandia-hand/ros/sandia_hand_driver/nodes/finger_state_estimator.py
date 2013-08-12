#!/usr/bin/env python
import roslib; roslib.load_manifest('sandia_hand_driver')
import rospy, sys, math
from sandia_hand_msgs.msg import RawFingerState, RawMoboState, \
                                 RawPalmState, CalFingerState
from sandia_hand_msgs.srv import *

# todo: at some point, consider porting this to c++ for speed,
# or even stuffing it into the driver node. easy to hack on it here though

class FingerStateEstimator:
  def __init__(self, argv):
    rospy.init_node('finger_state_estimator')
    self.cfs = CalFingerState()
    self.finger_name = rospy.get_param("~name", "index")
    self.finger_get_params = rospy.ServiceProxy('get_parameters',
                                           sandia_hand_msgs.srv.GetParameters)
    print "waiting for finger get_parameter service to become available..."
    rospy.wait_for_service("get_parameters")
    response = self.finger_get_params()
    fp = response.parameters
    #self.finger_params = self.finger_get_params()
    #print "got parameters: " + str(self.finger_params)
    self.accel_bias  = [0] * 9
    self.accel_scale = [1023] * 9
    scales = ["mm_accel_scale_x", "mm_accel_scale_y", "mm_accel_scale_z",
              "pp_accel_scale_x", "pp_accel_scale_y", "pp_accel_scale_z",
              "dp_accel_scale_x", "dp_accel_scale_y", "dp_accel_scale_z"]
    biases = ["mm_accel_bias_x", "mm_accel_bias_y", "mm_accel_bias_z",
              "pp_accel_bias_x", "pp_accel_bias_y", "pp_accel_bias_z",
              "dp_accel_bias_x", "dp_accel_bias_y", "dp_accel_bias_z"]
    for axis in xrange(0,9):
      self.accel_bias[axis]  = next(x.i_val for x in fp if x.name == biases[axis])
      self.accel_scale[axis] = next(x.i_val for x in fp if x.name == scales[axis])
    print "accel bias: " + str(self.accel_bias)
    print "accel scale: " + str(self.accel_scale)
    self.finger_pub = rospy.Publisher("cal_state", CalFingerState)
    self.finger_sub = rospy.Subscriber("raw_state", 
                                       RawFingerState, self.finger_cb)

  def simplify_angle(self, x):
    if x < -3.1415:
      return x + 2 * 3.1415 
    elif x > 3.1415:
      return x - 2 * 3.1415
    else:
      return x

  def calibrate_accel(self, raw_accel, ofs):
    #mag = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
    #if mag == 0:
    #  mag = 1 # we're hosed
    calib_accel = [0, 0, 0]
    for axis in xrange(0,3):
      calib_accel[axis] = float(raw_accel[axis] - \
                                self.accel_bias[axis + ofs]) / \
                       self.accel_scale[axis + ofs]
    return calib_accel

  def finger_cb(self, msg):
    ########################################################################
    # calcluate joint angles based on accelerometers
    mma = self.calibrate_accel(msg.mm_accel, 0)
    #if (finger_idx == 3):
    #  print [msg.mm_accel, mma]
    ppa = self.calibrate_accel(msg.pp_accel, 3)
    dpa = self.calibrate_accel(msg.dp_accel, 6)
    self.cfs.joints_inertial_variance = [1e6, 1e6, 1e6] # no bueno
    #print [dpa, ppa, mma]
    #print msg.mm_accel, msg.pp_accel, msg.dp_accel
    # first, estimate the easy one: the distal joint.
    j2 = math.atan2(ppa[2], ppa[0]) - math.atan2(dpa[2], dpa[0])
    self.cfs.joints_inertial_variance[2] = 0.1 # todo
    self.cfs.joints_inertial[2] = j2
    # next, estimate the proximal joint
    x0 = mma[0]
    y0 = mma[1]
    z0 = mma[2]
    x2 = ppa[0]
    y2 = ppa[1]
    z2 = ppa[2]
    det_j0 = x0*x0 + y0*y0 - y2*y2
    j0_sol = []
    if det_j0 > 0:
      j0_sol = [math.atan2(y2,  math.sqrt(det_j0)) - math.atan2(y0,-x0), \
                math.atan2(y2, -math.sqrt(det_j0)) - math.atan2(y0,-x0)]
    j0_valid = []
    for j0 in j0_sol:
      x = self.simplify_angle(j0)
      if self.finger_name != "thumb":
        if x > -1.6 and x < 1.6:
          j0_valid.append(x)
      else: # thumb has more severe constraints (fortunately)
        if x > -0.1 and x < 1.6:
          j0_valid.append(x)
    if len(j0_valid) == 1:
      j0 = j0_valid[0]
      self.cfs.joints_inertial[0] = j0
      self.cfs.joints_inertial_variance[0] = 0.1 # it's finite
      # finally, estimate the middle joint against both measurement
      # singularities
      gamma = (x0*math.cos(j0) + y0*math.sin(j0))
      det_j1_z = gamma**2 + z0*z0 - z2*z2
      j1_valid_z = []
      if det_j1_z > 0:
        j1_sol = [math.atan2(z2, math.sqrt(det_j1_z))-math.atan2(z0,gamma),\
                  math.atan2(z2,-math.sqrt(det_j1_z))-math.atan2(z0,gamma)]
        for j1 in j1_sol:
          x = -self.simplify_angle(j1) # flip sign to change to hand frame...
          if x > -1.6 and x < 1.6:
            j1_valid_z.append(x)
      # try it against the other measurement singularity now
      j1_valid_x = []
      det_j1_x = gamma**2 + z0*z0 - x2*x2
      if det_j1_x > 0:
        j1_sol = [math.atan2(x2, math.sqrt(det_j1_x))-math.atan2(gamma,-z0),\
                  math.atan2(x2,-math.sqrt(det_j1_x))-math.atan2(gamma,-z0)]
        for j1 in j1_sol:
          x = -self.simplify_angle(j1) # flip sign to change to hand frame...
          if x > -1.6 and x < 1.6:
            j1_valid_x.append(x)

      # the "right" answer for middle joint will emerge from both calculations
      # if x or z calculations are hosed, take the other
      j1_winner = 0
      j1_valid = False
      if   len(j1_valid_x) == 0 and len(j1_valid_z) == 1:
        j1_winner = j1_valid_z[0]
        j1_valid = True
      elif len(j1_valid_z) == 0 and len(j1_valid_x) == 1:
        j1_winner = j1_valid_x[0]
        j1_valid = True
      else: 
        for j1_x in j1_valid_x:
          for j1_z in j1_valid_z:
            if abs(j1_x - j1_z) < 0.001:
              j1_winner = j1_x
              j1_valid = True

      if j1_valid:
        self.cfs.joints_inertial[1] = j1_winner
        self.cfs.joints_inertial_variance[1] = 0.1 # it's finite

      #if finger_idx == 3:
      #  print [j1_valid_x, j1_valid_z]

    #if finger_idx == 3:
    #  print "j0: %.3f   %.3f" % (j0[0], j0[1])
    #  print j0_cand
      #print "  %.3f  %.3f" % (self.simplify_angle(j0[0]), self.simplify_angle(j0[1]))
      #print "    det: %.3f" % det
    ########################################################################
    # calculate joint angles based on hall sensor offsets
    H2R = 3.14159 * 2.0 / 36.0  # hall state to radians: 18 pole pairs
    R0_INV = 1.0 / 231.0
    R1_INV = 1.0 / 196.7
    R2_INV = 1.0 / 170.0
    CAPSTAN_RATIO = 0.89
    self.cfs.joints_hall[0] = -H2R * R0_INV *   msg.hall_pos[0]
    self.cfs.joints_hall[1] =  H2R * R1_INV * ( msg.hall_pos[1] + \
                                                CAPSTAN_RATIO*msg.hall_pos[0])
    self.cfs.joints_hall[2] =  H2R * R2_INV * ( msg.hall_pos[2] - \
                                               msg.hall_pos[1] - \
                                               CAPSTAN_RATIO*2*msg.hall_pos[0])
    self.finger_pub.publish(self.cfs)
    #if finger_idx == 0:
    #  print "  %.3f" % (j2*180./3.1415)

  def mobo_cb(self, msg):
    pass

  def palm_cb(self, msg):
    pass

############################################################################
if __name__ == '__main__':
  fse = FingerStateEstimator(rospy.myargv())
  rospy.spin()
