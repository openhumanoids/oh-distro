# Pan the head around using the 3 neck DOF, for example to build a map
import math
import drcargs
import bot_core as lcmbotcore

from director.utime import getUtime
from director import lcmUtils
import director.tasks.robottasks as rt


class HeadSurveyPattern(rt.AsyncTask):

    headSweepTime = 3.0

    lowerPitchAngles = [45 ,  35, 45, 35, 45, 35]
    yawAngles =        [-15, -15,  0,  0, 15, 15]
    upperPitchAngles = [  0,   0,  0,  0,  0,  0]

    #@staticmethod
    #def getDefaultProperties(properties):

    def run(self):
        #print self.lowerPitchAngles

        for i in range(len(self.lowerPitchAngles)):
             self.statusMessage = 'lowerNeckPitch: ' + str(self.lowerPitchAngles[i]) + ', neckYaw: ' + str(self.yawAngles[i]) + ', upperNeckPitch: ' + str(self.upperPitchAngles[i])
             self.publishAngle(self.lowerPitchAngles[i], self.yawAngles[i], self.upperPitchAngles[i])
             yield rt.DelayTask(delayTime=self.headSweepTime).run()

    @staticmethod
    def publishAngle(lowerNeckPitch, neckYaw, upperNeckPitch):
        jointGroups = drcargs.getDirectorConfig()['teleopJointGroups']
        jointGroupNeck = filter(lambda group: group['name'] == 'Neck', jointGroups)
        if (len(jointGroupNeck) == 1):
            neckJoints = jointGroupNeck[0]['joints']
        else:
            return
        m = lcmbotcore.joint_angles_t()
        m.utime = getUtime()
        m.num_joints = 3
        m.joint_name = [ neckJoints[0], neckJoints[1], neckJoints[2] ]
        m.joint_position = [ math.radians(lowerNeckPitch), math.radians(neckYaw), math.radians(upperNeckPitch)]
        lcmUtils.publish('DESIRED_NECK_ANGLES', m)