#!/usr/bin/env python
#
# Filters out the lower arm joints of the COMMITTED_ROBOT_PLAN and 
# passes them to the Forearm Controller

import lcm
import bot_core as lcmbotcore
import drc as lcmdrc

def onRobotPlan(channel, data):
    msg = lcmdrc.robot_plan_t.decode(data)
    plan = msg.plan[-1] # last pose

    rightWristRollIndex = plan.joint_name.index("rightWristRoll")
    rightWristPitchIndex = plan.joint_name.index("rightWristPitch")
    rightForearmYawIndex = plan.joint_name.index("rightForearmYaw")

    rightWristRollRad = plan.joint_position[rightWristRollIndex]
    rightWristPitchRad = plan.joint_position[rightWristPitchIndex]
    rightForearmYawRad = plan.joint_position[rightForearmYawIndex]

    print "Commanding forearm yaw to " + str(rightForearmYawRad)
    print "Commanding wrist roll to " + str(rightWristRollRad)
    print "Commanding wrist pitch to " + str(rightWristPitchRad)
    sendWristCommand(msg.utime, rightForearmYawRad, rightWristRollRad, rightWristPitchRad)


def sendWristCommand(utime, forearmYaw, roll, pitch):
	msg = lcmbotcore.joint_angles_t()
	msg.utime = utime
	msg.joint_name = [ "rightForearmYaw", "rightWristRoll", "rightWristPitch" ]
	msg.num_joints = len(msg.joint_name)
	msg.joint_position = [ forearmYaw, roll, pitch ]

	theLcm.publish("DESIRED_FOREARM_ANGLES", msg.encode())


theLcm = lcm.LCM()
theLcm.subscribe("COMMITTED_ROBOT_PLAN", onRobotPlan)

while True:
	theLcm.handle()
