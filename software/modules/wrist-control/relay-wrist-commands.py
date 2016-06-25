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

    leftWristRollIndex = plan.joint_name.index("leftWristRoll")
    leftWristPitchIndex = plan.joint_name.index("leftWristPitch")
    leftForearmYawIndex = plan.joint_name.index("leftForearmYaw")

    leftWristRollRad = plan.joint_position[leftWristRollIndex]
    leftWristPitchRad = plan.joint_position[leftWristPitchIndex]
    leftForearmYawRad = plan.joint_position[leftForearmYawIndex]

    print "Commanding forearm yaw to " + str(leftForearmYawRad)
    print "Commanding wrist roll to " + str(leftWristRollRad)
    print "Commanding wrist pitch to " + str(leftWristPitchRad)
    sendWristCommand(msg.utime, leftForearmYawRad, leftWristRollRad, leftWristPitchRad)


def sendWristCommand(utime, forearmYaw, roll, pitch):
	msg = lcmbotcore.joint_angles_t()
	msg.utime = utime
	msg.joint_name = [ "leftForearmYaw", "leftWristRoll", "leftWristPitch" ]
	msg.num_joints = len(msg.joint_name)
	msg.joint_position = [ forearmYaw, roll, pitch ]

	theLcm.publish("DESIRED_FOREARM_ANGLES", msg.encode())


theLcm = lcm.LCM()
theLcm.subscribe("COMMITTED_ROBOT_PLAN", onRobotPlan)

while True:
	theLcm.handle()
