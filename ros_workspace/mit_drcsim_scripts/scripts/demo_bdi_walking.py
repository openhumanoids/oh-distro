#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('control_mode_switch')
import rospy
import time
from atlas_msgs.msg import AtlasCommand, AtlasSimInterfaceCommand, AtlasSimInterfaceState, AtlasState, AtlasBehaviorStepData
from sensor_msgs.msg import Imu
import PyKDL
from tf_conversions import posemath

class Demo:

    def __init__(self):
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

        # Step 4: Request BDI walk
        walk_msg = AtlasSimInterfaceCommand()
        # Always insert current time
        walk_msg.header.stamp = rospy.Time.now()
        # Tell it to walk
        walk_msg.behavior = walk_msg.WALK
        walk_msg.walk_params.use_demo_walk = False
        # Fill in some steps
        for i in range(4):
            step_queue = AtlasBehaviorStepData()
            # Steps are indexed starting at 1
            step_queue.step_index = i+1
            # 0 = left, 1 = right
            step_queue.foot_index = i%2
            # 0.3 is a good number
            step_queue.swing_height = 0.3
            # 0.63 is a good number
            step_queue.duration = 0.63
            # We'll specify desired foot poses in ego-centric frame then
            # transform them into the robot's world frame.
            # Match feet so that we end with them together
            step_queue.pose.position.x = (1+i/2)*0.25
            # Step 0.15m to either side of center, alternating with feet
            step_queue.pose.position.y = 0.15 if (i%2==0) else -0.15
            step_queue.pose.position.z = 0.0
            # Point those feet straight ahead
            step_queue.pose.orientation.x = 0.0
            step_queue.pose.orientation.y = 0.0
            step_queue.pose.orientation.z = 0.0
            step_queue.pose.orientation.w = 1.0
            # Transform this foot pose according to robot's
            # current estimated pose in the world, which is a combination of IMU
            # and internal position estimation.
            # http://www.ros.org/wiki/kdl/Tutorials/Frame%20transformations%20%28Python%29
            f1 = posemath.fromMsg(step_queue.pose)
            f2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(self.imu_msg.orientation.x,
                                                       self.imu_msg.orientation.y,
                                                       self.imu_msg.orientation.z,
                                                       self.imu_msg.orientation.w),
                             PyKDL.Vector(self.asis_msg.pos_est.position.x,
                                          self.asis_msg.pos_est.position.y,
                                          self.asis_msg.pos_est.position.z))
            f = f2 * f1
            step_queue.pose = posemath.toMsg(f)
            walk_msg.walk_params.step_queue[i] = step_queue
        # Publish and give time to take effect
        print('[USER/BDI] Walking...')
        self.asic_pub.publish(walk_msg)
        time.sleep(6.0)

if __name__ == '__main__':
    rospy.init_node('control_mode_switch')
    d = Demo()
    d.demo()
