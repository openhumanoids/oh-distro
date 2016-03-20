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
from director.lcmframe import positionMessageFromFrame

import drc as lcmdrc

class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot

class ManualWalkingDemo(object):
    def __init__(self, robotStateModel, footstepsDriver, robotStateJointController, ikPlanner):
        self.footstepsDriver = footstepsDriver
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner

        # live operation flags
        self.isLeadingFootRight = False

        # Manual footsteps placement options
        self.numSteps = 4
        self.forwardStep = 0.35
        self.stepWidth = 0.25
        # IHMC params
        self.ihmcTransferTime = 1.0
        self.ihmcSwingTime = 1.0

    def testManualWalking(self, leadFoot=None):
        if (leadFoot is None):
            if self.isLeadingFootRight:
                leadFoot=self.ikPlanner.rightFootLink #'r_foot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'l_foot'

        feetMidPoint = self.footstepsDriver.getFeetMidPoint(self.robotStateModel)
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.manualFootstepsPlacement(leadFoot, startFeetMidPoint = feetMidPoint, nextDoubleSupportPose = startPose)


    def manualFootstepsPlacement(self, standingFootName, startFeetMidPoint = None, nextDoubleSupportPose = None):
        # Step 1: Footsteps placement
        footsteps = self.placeStepsManually(startFeetMidPoint)

        assert len(footsteps) > 0

        # Step 2: Send request to planner. It replies with complete footsteps plan message.
        self.sendFootstepPlanRequest(footsteps, nextDoubleSupportPose)


    def placeStepsManually(self, startFeetMidPoint = None):
        
        # Place footsteps on the ground manually (no planning)
        # The number of footsteps, width and forward distance between steps are given by user.
        footsteps = []

        startFeetMidPointPos = startFeetMidPoint.GetPosition()

        contactPtsLeft, contactPtsRight = self.footstepsDriver.getContactPts()
        contactPtsMid = np.mean(contactPtsRight, axis=0) # mid point on foot relative to foot frame

        forward = 0
        for i in range(self.numSteps):
            if i == 0:
                correctionFootToMidContact = contactPtsMid[0]
            else:
                correctionFootToMidContact = 0

            if i < (self.numSteps-1):
                forward = forward + (self.forwardStep / 2) - correctionFootToMidContact

            isLeadingFootLeft = not(self.isLeadingFootRight)
            if (i % 2 == 0 and isLeadingFootLeft) or (i % 2 != 0 and self.isLeadingFootRight):
                width = self.stepWidth / 2
            else:
                width = -self.stepWidth / 2

            stepPose = transformUtils.frameFromPositionAndRPY([forward, width, startFeetMidPointPos[2]], [0,0,0])

            nextTransform = transformUtils.copyFrame(startFeetMidPoint)
            nextTransform.PreMultiply()
            nextTransform.Concatenate(stepPose)

            if i % 2 == 0:
                footsteps.append(Footstep(nextTransform,self.isLeadingFootRight))
            else:
                footsteps.append(Footstep(nextTransform,isLeadingFootLeft))

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

class ManualWalkingTaskPanel(TaskUserPanel):

    def __init__(self, manualWalkingDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.manualWalkingDemo = manualWalkingDemo

        self.addDefaultProperties()
        self.addButtons()


    def addButtons(self):
        self.addManualButton('Footsteps Plan', self.manualWalkingDemo.testManualWalking)
        self.addManualButton('EXECUTE Plan', self.manualWalkingDemo.executePlan)

    def addDefaultProperties(self):
        self.params.addProperty('Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Left','Right']))
        self.params.addProperty('Num Steps', 6, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Forward Step', 0.35, attributes=om.PropertyAttributes(decimals=2, minimum=-0.5, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Step Width', 0.25, attributes=om.PropertyAttributes(decimals=2, minimum=0.15, maximum=0.6, singleStep=0.01))
        self.params.addProperty('IHMC Transfer Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.25, maximum=2.0, singleStep=0.01))
        self.params.addProperty('IHMC Swing Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.6, maximum=1.5, singleStep=0.01))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.manualWalkingDemo.testManualWalking()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.manualWalkingDemo.isLeadingFootRight = False
        else:
            self.manualWalkingDemo.isLeadingFootRight = True
    
        self.manualWalkingDemo.numSteps = self.params.getProperty('Num Steps')
        self.manualWalkingDemo.forwardStep = self.params.getProperty('Forward Step')
        self.manualWalkingDemo.stepWidth = self.params.getProperty('Step Width')
        self.manualWalkingDemo.ihmcTransferTime = self.params.getProperty('IHMC Transfer Time')
        self.manualWalkingDemo.ihmcSwingTime = self.params.getProperty('IHMC Swing Time')

