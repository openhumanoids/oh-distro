import math

from director import lcmUtils
from director.utime import getUtime

import drc as lcmdrc
import bot_core as lcmbotcore

class ValkyrieDriver(object):

    def __init__(self, ikPlanner):
        self.ikPlanner = ikPlanner

    def sendWholeBodyCommand(self, wholeBodyMode):
        msg = lcmdrc.int64_stamped_t()
        msg.utime = getUtime()
        msg.data = wholeBodyMode
        lcmUtils.publish('IHMC_CONTROL_MODE_COMMAND', msg)

    def setNeckPitch(self, neckPitchDegrees):
        assert neckPitchDegrees <= 45 and neckPitchDegrees >= 0

        msg = lcmbotcore.joint_angles_t()
        msg.utime = getUtime()
        msg.num_joints = 1
        msg.joint_name = ["lowerNeckPitch"]
        msg.joint_position = [math.radians(neckPitchDegrees)]
        lcmUtils.publish("DESIRED_NECK_ANGLES", msg)

    def sendParkNeckCommand(self):
        msg = lcmbotcore.joint_angles_t()
        msg.utime = getUtime()
        msg.joint_name = ["lowerNeckPitch", "neckYaw", "upperNeckPitch"]
        msg.joint_position = [0] * len(msg.joint_name)
        msg.num_joints = len(msg.joint_name)
        lcmUtils.publish("DESIRED_NECK_ANGLES", msg)

    def sendHandCommand(self, side, thumbRoll, thumbPitch1, thumbPitch2, indexFingerPitch, middleFingerPitch, pinkyPitch):
        assert side in ["left", "right"]
        assert thumbRoll >= 0.0 and thumbRoll <= 1.0
        assert thumbPitch1 >= 0.0 and thumbPitch1 <= 1.0
        assert thumbPitch2 >= 0.0 and thumbPitch2 <= 1.0
        assert indexFingerPitch >= 0.0 and indexFingerPitch <= 1.0
        assert middleFingerPitch >= 0.0 and middleFingerPitch <= 1.0
        assert pinkyPitch >= 0.0 and pinkyPitch <= 1.0

        msg = lcmbotcore.joint_angles_t()
        msg.joint_name = [side + "IndexFingerMotorPitch1", side + "MiddleFingerMotorPitch1", side + "PinkyMotorPitch1", side + "ThumbMotorPitch1", side + "ThumbMotorPitch2", side + "ThumbMotorRoll"]
        msg.num_joints = len(msg.joint_name)
        msg.joint_position = [indexFingerPitch, middleFingerPitch, pinkyPitch, thumbPitch1, thumbPitch2, thumbRoll]

        lcmUtils.publish("DESIRED_HAND_ANGLES", msg)


def init(ikPlanner):

    global driver
    driver = ValkyrieDriver(ikPlanner)

    return driver
