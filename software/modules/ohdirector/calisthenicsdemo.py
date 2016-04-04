import os
import functools
import numpy as np

from director.tasks.taskuserpanel import TaskUserPanel
import director.tasks.robottasks as rt
from director import objectmodel as om
from director import visualization as vis
from director import lcmUtils
from director import segmentation
from director import transformUtils
from director import footstepsdriver
from director.utime import getUtime
from director.lcmframe import positionMessageFromFrame

import drc as lcmdrc
import ihmc as lcmihmc


class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot

class CalisthenicsDemo(object):
    def __init__(self, robotStateModel, footstepsDriver, robotStateJointController, ikPlanner):
        self.footstepsDriver = footstepsDriver
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner

    def testCalisthenics(self):
        feetMidPoint = self.footstepsDriver.getFeetMidPoint(self.robotStateModel)
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        # Step 1: Footsteps placement
        footsteps = self.placeStepsManually(feetMidPoint)

        assert len(footsteps) > 0

        # Step 2: Send request to planner. It replies with complete footstep plan message.
        self.sendFootstepPlanRequest(footsteps, startPose)

    def getSimpleFootstepsFolder(self):
        obj = om.findObjectByName('simple footsteps')
        if obj is None:
            obj = om.getOrCreateContainer('simple footsteps')
            obj.setProperty('Visible', False)
            om.collapse(obj)
        return obj

    def placeStepsManually(self, startFeetMidPoint = None):
        # Place footsteps on the ground manually (no planning)
        # The number of footsteps, width, forward distance and turning angle between steps are given by user.
        footsteps = []

        contactPtsLeft, contactPtsRight = self.footstepsDriver.getContactPts()
        contactPtsMid = np.mean(contactPtsRight, axis=0) # mid point on foot relative to foot frame

        midLineFrame = transformUtils.copyFrame(startFeetMidPoint)

        simpleFootstepsFolder = self.getSimpleFootstepsFolder()
        map(om.removeFromObjectModel, simpleFootstepsFolder.children())

        for i in range(self.numSteps):
            if i == 0:
                correctionFootToMidContact = contactPtsMid[0]
            else:
                correctionFootToMidContact = 0

            if i < (self.numSteps-1):
                forwardDistance = (self.forwardStep / 2) - correctionFootToMidContact
            else:
                forwardDistance = 0

            if (i % 2 == 0 and not self.isLeadingFootRight) or (i % 2 != 0 and self.isLeadingFootRight):
                leftRightShift = self.stepWidth / 2
            else:
                leftRightShift = -self.stepWidth / 2

            if i < (self.numSteps-1):
                nextTurnAngle = self.turnAngle
            else:
                nextTurnAngle = 0

            # Determine the forward motion of a frame along the center line between the feet:
            forwardMotionTransform = transformUtils.frameFromPositionAndRPY([forwardDistance, 0, 0], [0,0, nextTurnAngle])
            midLineFrame.PreMultiply()
            midLineFrame.Concatenate(forwardMotionTransform)
            #vis.showFrame(transformUtils.copyFrame(midLineFrame), "midLineFrame")

            # Shift the feet left or right as required
            nextTransform = transformUtils.copyFrame(midLineFrame)
            leftRightTransform = transformUtils.frameFromPositionAndRPY([0, leftRightShift, 0], [0,0,0])
            nextTransform.PreMultiply()
            nextTransform.Concatenate(leftRightTransform)
            vis.showFrame(transformUtils.copyFrame(nextTransform), "nextTransform", parent=simpleFootstepsFolder, visible=simpleFootstepsFolder.getProperty('Visible'))

            if i % 2 == 0:
                footsteps.append(Footstep(nextTransform,self.isLeadingFootRight))
            else:
                footsteps.append(Footstep(nextTransform,not(self.isLeadingFootRight)))

        return footsteps


    def sendFootstepPlanRequest(self, footsteps, nextDoubleSupportPose):
        goalSteps = []

        for i, footstep in enumerate(footsteps):
            step_t = footstep.transform

            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot =  footstep.is_right_foot
            # Set ihmc parameters
            default_step_params = self.footstepsDriver.getDefaultStepParams()
            default_step_params.ihmc_transfer_time = self.ihmcTransferTime
            default_step_params.ihmc_swing_time = self.ihmcSwingTime
            step.params = default_step_params

            goalSteps.append(step)

        request = self.footstepsDriver.constructFootstepPlanRequest(nextDoubleSupportPose)
        request.num_goal_steps = len(goalSteps)
        request.goal_steps = goalSteps

        # force correct planning parameters:
        request.params.leading_foot = goalSteps[0].is_right_foot
        request.params.planning_mode = lcmdrc.footstep_plan_params_t.MODE_SPLINE
        request.params.map_mode = lcmdrc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS
        request.params.max_num_steps = len(goalSteps)
        request.params.min_num_steps = len(goalSteps)
        request.default_step_params = default_step_params

        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)

    def executePlan(self):
        self.footstepsDriver.commitFootstepPlan(self.footstepsDriver.lastFootstepPlan)


    def makeFootFrame(self, side):
        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        footTransform = self.robotStateModel.getLinkFrame(footLink)
        vis.showFrame(footTransform, "right foot goal")

    def makeLeftFootFrame(self):
        self.makeFootFrame("left")

    def makeRightFootFrame(self):
        self.makeFootFrame("right")

    def moveRightFootToFrame(self):
        self.moveFootToFrame("right")


    def moveFootToFrame(self, side):
        if side == 'left':
            footFrame = om.findObjectByName('right foot goal')
        else:
            footFrame = om.findObjectByName('right foot goal')

        self.moveFootToFrame(footFrame.transform, side)


    def getCurrentRelativeFootFrame(self, side = 'right'):
        # Get the frame between the standing foot (otherSide) and the floating foot (side)
        side = 'right'
        otherSide = 'left'

        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        footTransform = self.robotStateModel.getLinkFrame(footLink)
        vis.updateFrame(footTransform, "current foot frame")

        otherFootLink = self.ikPlanner.leftFootLink if otherSide == 'left' else self.ikPlanner.rightFootLink
        otherFootTransform = self.robotStateModel.getLinkFrame(otherFootLink)
        vis.updateFrame(otherFootTransform, "other foot frame")

        relativeFootTransform = transformUtils.copyFrame(footTransform)
        relativeFootTransform.PreMultiply()
        relativeFootTransform.Concatenate(otherFootTransform.GetLinearInverse())

        [ pos, quat ] = transformUtils.poseFromTransform(relativeFootTransform)

        print 'Relative Transform from', footLink, 'to', otherFootLink
        print pos
        print quat


    def moveFootToSkater(self):
        side = 'right'
        pos = [-0.79294841, -0.1598023,  0.4183836 ]
        quat = [ 0.63925192, -0.00519966,  0.76897871, -0.00129613]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), side)

    def moveFootToPreSkater(self):
        side = 'right'
        pos = [-0.45855504, -0.15881477,  0.19467936]
        quat = [  8.49938853e-01,  -3.58606857e-03,   5.26868563e-01,   7.76788522e-04]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), side)

    def moveFootToJustOffGround(self):
        side = 'right'
        pos = [ 0.0, -0.24, 0.05]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), side)

    def moveFootToOnGround(self):
        side = 'right'
        pos = [ 0.0, -0.24, 0.00]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), side)

    def moveFootToBelowGround(self):
        side = 'right'
        pos = [ 0.0, -0.24, -0.02]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), side)

    def moveFootToRelativeFrame(self, relativeFootTransform, side = 'right'):
        # move side to this transfrom relative to the otherSide (which we are standing on)       
        side = 'right'
        otherSide = 'left'

        otherFootLink = 'right' if otherSide == 'left' else 'left'

        otherFootLink = self.ikPlanner.leftFootLink if otherSide == 'left' else self.ikPlanner.rightFootLink
        otherFootTransform = self.robotStateModel.getLinkFrame(otherFootLink)
        vis.updateFrame(otherFootTransform, "other foot frame")

        goalFootTransform = transformUtils.copyFrame(otherFootTransform)
        goalFootTransform.PreMultiply()
        goalFootTransform.Concatenate(relativeFootTransform)

        self.moveFootToFrame(goalFootTransform, side)

    def moveFootToFrame(self, goalFootTransform, side):
        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        currenFootTransform = self.robotStateModel.getLinkFrame(footLink)

        msg = lcmihmc.foot_pose_packet_message_t()
        msg.utime = getUtime()
        [msg.position, msg.orientation] = transformUtils.poseFromTransform(goalFootTransform)

        # Rule of thumb speed limit: 10cm/sec
        distanceToMoveFoot = np.linalg.norm(tuple(np.subtract(currenFootTransform.GetPosition(), goalFootTransform.GetPosition() )) )
        msg.trajectory_time = distanceToMoveFoot*10.0

        if side == 'left':
            msg.robot_side = 0
            lcmUtils.publish("DESIRED_LEFT_FOOT_POSE", msg)
        else:
            msg.robot_side = 1
            lcmUtils.publish("DESIRED_RIGHT_FOOT_POSE", msg)


class CalisthenicsTaskPanel(TaskUserPanel):

    def __init__(self, calisthenicsDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.calisthenicsDemo = calisthenicsDemo

        self.addDefaultProperties()
        self.addButtons()
        self.autoQuery = True

    def addButtons(self):
        self.addManualButton('Plan Footsteps', self.calisthenicsDemo.testCalisthenics)
        self.addManualButton('Execute Plan', self.calisthenicsDemo.executePlan)

        self.addManualSpacer()
        self.addManualButton('Set Straight Defaults', self.setStraightDefaults)
        self.addManualButton('Create Left Frame', self.calisthenicsDemo.makeLeftFootFrame)       
        self.addManualButton('Create Right Frame', self.calisthenicsDemo.makeRightFootFrame)
        self.addManualButton('Move Right Foot', self.calisthenicsDemo.moveRightFootToFrame)

        self.addManualSpacer()
        self.addManualButton('RFoot Below', self.calisthenicsDemo.moveFootToBelowGround)
        self.addManualButton('RFoot Just Above', self.calisthenicsDemo.moveFootToJustOffGround)
        self.addManualButton('RFoot Pre Skater', self.calisthenicsDemo.moveFootToPreSkater)
        self.addManualButton('RFoot Skater', self.calisthenicsDemo.moveFootToSkater)

    def setStraightDefaults(self, makeQuery = True):
        self.autoQuery = False

        # Manual footsteps placement options
        self.params.setProperty('Leading Foot', 1) # Right
        self.params.setProperty('Num Steps', 6)
        self.params.setProperty('Forward Step', 0.35)
        self.params.setProperty('Turn Angle', 0.0) # degrees
        self.params.setProperty('Step Width', 0.25)
        # IHMC params
        self.params.setProperty('IHMC Transfer Time', 1.0)
        self.params.setProperty('IHMC Swing Time', 1.0)

        self._syncProperties()
        if (makeQuery):
            self.calisthenicsDemo.testCalisthenics()
        self.autoQuery = True

    def addDefaultProperties(self):
        self.params.addProperty('Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Left','Right']))
        self.params.addProperty('Num Steps',  6, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Forward Step', 0.35 , attributes=om.PropertyAttributes(decimals=2, minimum=-0.6, maximum=0.6, singleStep=0.01))
        self.params.addProperty('Turn Angle', 0.0, attributes=om.PropertyAttributes(decimals=2, minimum=-40, maximum=40, singleStep=1))
        self.params.addProperty('Step Width', 0.25, attributes=om.PropertyAttributes(decimals=2, minimum=0.15, maximum=0.6, singleStep=0.01))
        self.params.addProperty('IHMC Transfer Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.25, maximum=2.0, singleStep=0.01))
        self.params.addProperty('IHMC Swing Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.6, maximum=1.5, singleStep=0.01))

        self.setStraightDefaults(False)
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()

        if (self.autoQuery):
            self.calisthenicsDemo.testCalisthenics()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.calisthenicsDemo.isLeadingFootRight = False
        else:
            self.calisthenicsDemo.isLeadingFootRight = True
    
        self.calisthenicsDemo.numSteps = self.params.getProperty('Num Steps')
        self.calisthenicsDemo.forwardStep = self.params.getProperty('Forward Step')
        self.calisthenicsDemo.turnAngle = self.params.getProperty('Turn Angle')
        self.calisthenicsDemo.stepWidth = self.params.getProperty('Step Width')
        self.calisthenicsDemo.ihmcTransferTime = self.params.getProperty('IHMC Transfer Time')
        self.calisthenicsDemo.ihmcSwingTime = self.params.getProperty('IHMC Swing Time')

