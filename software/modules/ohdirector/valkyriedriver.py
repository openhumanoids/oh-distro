import os
import vtkAll as vtk
import math
import numpy as np
from collections import deque

from director import transformUtils
from director import lcmUtils
from director.timercallback import TimerCallback
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ioUtils
from director.utime import getUtime
import time

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

    def sendParkNeckCommand(self):
        msg = lcmbotcore.joint_angles_t()
        msg.utime = getUtime()
        msg.joint_name = ["lowerNeckPitch", "neckYaw", "upperNeckPitch"]
        msg.joint_position = [0] * len(msg.joint_name)
        msg.num_joints = len(msg.joint_name)
        lcmUtils.publish("DESIRED_NECK_ANGLES", msg)

    def sendHandCommand(self, side="right"):
        assert side in ["left", "right"]

        msg = lcmbotcore.joint_angles_t()
        msg.joint_name = [ side + "IndexFingerMotorPitch1", side + "MiddleFingerMotorPitch1", side + "PinkyMotorPitch1", side + "ThumbMotorPitch1", side + "ThumbMotorPitch2", side + "ThumbMotorRoll"]
        msg.num_joints = len(msg.joint_name)
        msg.joint_position = [0, 0, 0, 0, 0, 0] #1]

        lcmUtils.publish("DESIRED_HAND_ANGLES", msg)


def init(ikPlanner):

    global driver
    driver = ValkyrieDriver(ikPlanner)

    return driver
