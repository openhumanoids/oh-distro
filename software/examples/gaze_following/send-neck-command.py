#!/usr/bin/env python

import lcm
import bot_core as lcmbotcore
import time
import sys

myLCM = lcm.LCM()

msg = lcmbotcore.robot_state_t()
msg.num_joints = 3
msg.joint_name = ["lowerNeckPitch", "neckYaw", "upperNeckPitch"]
msg.joint_velocity = [0, 0, 0]
msg.joint_effort = [0, 0, 0]

if len(sys.argv) > 1:
   msg.joint_position = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
else:  # Demo
   msg.joint_position = [0., 0., 0.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
   time.sleep(4)
   msg.joint_position = [1., 0., 0.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
   time.sleep(4)
   msg.joint_position = [1., 0., -1.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
   time.sleep(4)
   msg.joint_position = [1., -0.3, 0.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
   time.sleep(4)
   msg.joint_position = [0., 0.3, 0.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())
   time.sleep(4)
   msg.joint_position = [0., 0., 0.]
   myLCM.publish("JOINT_POSITION_GOAL", msg.encode())