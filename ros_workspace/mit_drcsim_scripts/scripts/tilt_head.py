#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('mit_drcsim_scripts')
import rospy
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath
import sys


class Demo:

    def __init__(self):
        self.neck_angle_desired =0.0
        self.NUM_JOINTS = 28
        # Latest message from /atlas/atlas_sim_interface_state
        self.asis_msg = None
        # Latest message from /atlas/imu
        self.imu_msg = None

        # Set up publishers / subscribers
        self.ac_pub = rospy.Publisher('atlas/atlas_command', AtlasCommand)
        self.asic_pub = rospy.Publisher('atlas/atlas_sim_interface_command', 
          AtlasSimInterfaceCommand)
        self.asis_sub = rospy.Subscriber('atlas/atlas_sim_interface_state', 
          AtlasSimInterfaceState, self.state_cb)
        self.imu_sub = rospy.Subscriber('atlas/imu', 
          Imu, self.imu_cb)
        # Wait for subscribers to hook up, lest they miss our commands
        time.sleep(2.0)

    def state_cb(self, msg):
        self.asis_msg = msg

    def imu_cb(self, msg):
        self.imu_msg = msg

    def demo(self):
        # Step 3: Move the arms and head a little (not too much; don't want to fall
        # over)
        slight_movement_msg = AtlasCommand()
        # Always insert current time
        slight_movement_msg.header.stamp = rospy.Time.now()
        # Start with 0.0 and set values for the joints that we want to control
        slight_movement_msg.position = [0.0] * self.NUM_JOINTS
        # set angle. 1 down. 0 flat. -1 up
        slight_movement_msg.position[AtlasState.neck_ry] = self.neck_angle_desired
        slight_movement_msg.velocity = [0.0] * self.NUM_JOINTS
        slight_movement_msg.effort = [0.0] * self.NUM_JOINTS
        slight_movement_msg.kp_position = [20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0]
        slight_movement_msg.ki_position = [0.0] * self.NUM_JOINTS
        slight_movement_msg.kd_position = [0.0] * self.NUM_JOINTS
        # Bump up kp_velocity to reduce the jerkiness of the transition
        slight_movement_msg.kp_velocity = [10.0] * self.NUM_JOINTS
        slight_movement_msg.i_effort_min = [0.0] * self.NUM_JOINTS
        slight_movement_msg.i_effort_max = [0.0] * self.NUM_JOINTS 
        # Set k_effort = [1] for the joints that we want to control.
        # BDI has control of the other joints
        slight_movement_msg.k_effort = [0] * self.NUM_JOINTS
        slight_movement_msg.k_effort[AtlasState.neck_ry] = 255
        # Publish and give time to take effect
        print('[USER/BDI] Command neck angle...')
        self.ac_pub.publish(slight_movement_msg)
        time.sleep(2.0)

if __name__ == '__main__':
    #print ("Script name: %s" % str(sys.argv[0]))
    rospy.init_node('tilt_head')
    d = Demo()
    if (len(sys.argv) > 1):
      print ("Setting neck to: %s" % str(sys.argv[1]))
      d.neck_angle_desired = float(sys.argv[1])
    else:
      print ("Syntax: rosrun <package_name> tilt_head 1.0")
      print ("append a joint angle for the neck")
      print ("Setting neck to: 0")
      d.neck_angle_desired = 0.0
    d.demo()
