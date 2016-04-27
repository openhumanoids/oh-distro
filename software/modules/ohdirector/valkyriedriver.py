import copy
import math

from director import lcmUtils
from director.utime import getUtime

import drc as lcmdrc
import bot_core as lcmbotcore

class ValkyrieDriver(object):

    def __init__(self, ikPlanner):
        self.ikPlanner = ikPlanner

        self.leftHandJoints = ["leftIndexFingerMotorPitch1", "leftMiddleFingerMotorPitch1", "leftPinkyMotorPitch1", "leftThumbMotorPitch1", "leftThumbMotorPitch2", "leftThumbMotorRoll"]
        self.rightHandJoints = ["rightIndexFingerMotorPitch1", "rightMiddleFingerMotorPitch1", "rightPinkyMotorPitch1", "rightThumbMotorPitch1", "rightThumbMotorPitch2", "rightThumbMotorRoll"]

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
        assert (thumbRoll >= 0.0 and thumbRoll <= 1.0) or thumbRoll == None # if not sending thumbRoll, set None (will be removed from message)
        assert thumbPitch1 >= 0.0 and thumbPitch1 <= 1.0
        assert thumbPitch2 >= 0.0 and thumbPitch2 <= 1.0
        assert indexFingerPitch >= 0.0 and indexFingerPitch <= 1.0
        assert middleFingerPitch >= 0.0 and middleFingerPitch <= 1.0
        assert pinkyPitch >= 0.0 and pinkyPitch <= 1.0

        # Remove thumbRoll if set to None
        handJoints = copy.deepcopy(self.leftHandJoints) if side == "left" else copy.deepcopy(self.rightHandJoints)
        if thumbRoll is None:
            for fingerJoint in handJoints:
                if "MotorRoll" in fingerJoint:
                    handJoints.remove(fingerJoint)

        msg = lcmbotcore.joint_angles_t()
        msg.joint_name = handJoints
        msg.num_joints = len(msg.joint_name)

        if msg.num_joints == 5:  # Excluding thumb roll
            msg.joint_position = [indexFingerPitch, middleFingerPitch, pinkyPitch, thumbPitch1, thumbPitch2]
        elif msg.num_joints == 6:
            msg.joint_position = [indexFingerPitch, middleFingerPitch, pinkyPitch, thumbPitch1, thumbPitch2, thumbRoll]

        lcmUtils.publish("DESIRED_HAND_ANGLES", msg)

    def openHand(self, side):
        '''
        Opens all pitch joints, doesn't move the thumb roll (Pseudo soft E-stop)
        '''
        assert side in ["left", "right"]
        self.sendHandCommand(side, thumbRoll=None, thumbPitch1=0.0, thumbPitch2=0.0, indexFingerPitch=0.0, middleFingerPitch=0.0, pinkyPitch=0.0)



def init(ikPlanner):

    global driver
    driver = ValkyrieDriver(ikPlanner)

    return driver
