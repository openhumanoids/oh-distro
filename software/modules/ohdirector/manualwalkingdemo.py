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
        self.leadingFootByUser = 'Left'

        # Manual footsteps placement options
        self.numSteps = 3
        self.forwardStep = 0.35
        self.forwardBalance = 0.05
        self.stepWidth = 0.25
        self.lateralShift = 0.0

    def testManualWalking(self, leadFoot=None):
        if (leadFoot is None):
            if self.leadingFootByUser == 'Right':
                leadFoot=self.ikPlanner.rightFootLink #'r_foot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'l_foot'

        feetMidPoint = self.footstepsDriver.getFeetMidPoint(self.robotStateModel)
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.manualFootstepsPlacement(leadFoot, removeFirstLeftStep = False, startFeetMidPoint = feetMidPoint, nextDoubleSupportPose = startPose)


    def manualFootstepsPlacement(self, standingFootName, removeFirstLeftStep = True, startFeetMidPoint = None, nextDoubleSupportPose = None):
        # Step 1: Footsteps placement
        footsteps = self.placeStepsManually(removeFirstLeftStep, startFeetMidPoint)

        assert len(footsteps) > 0

        # Step 2: Send request to planner. It replies with complete footsteps plan message.
        self.sendFootstepPlanRequest(footsteps, nextDoubleSupportPose)


    def placeStepsManually(self, removeFirstLeftStep = True, startFeetMidPoint = None):
        
        # Place footsteps an the ground manually (no planning)
        # The number of footsteps, width and forward distance between steps are given by user.
        footsteps = []

        startFeetMidPointPos = startFeetMidPoint.GetPosition()

        contact_pts_left, contact_pts_right = self.footstepsDriver.getContactPts()
        contact_pts_mid_left = np.mean(contact_pts_left, axis=0) # mid point on foot relative to foot frame
        contact_pts_mid_right = np.mean(contact_pts_right, axis=0) # mid point on foot relative to foot frame

        for i in range(self.numSteps):
            fRight = self.forwardStep*(i+1) - contact_pts_mid_right[0] + self.forwardBalance
            wRight = -self.stepWidth/2 + self.lateralShift*(i+1)
            fLeft = self.forwardStep*(i+1) - contact_pts_mid_right[0]
            wLeft = self.stepWidth/2 + self.lateralShift*(i+1)

            stepPoseRight = transformUtils.frameFromPositionAndRPY([fRight,wRight,startFeetMidPointPos[2]], [0,0,0])
            stepPoseLeft = transformUtils.frameFromPositionAndRPY([fLeft,wLeft,startFeetMidPointPos[2]], [0,0,0])

            nextRightTransform = transformUtils.copyFrame(startFeetMidPoint)
            nextRightTransform.PreMultiply()
            nextRightTransform.Concatenate(stepPoseRight)

            nextLeftTransform = transformUtils.copyFrame(startFeetMidPoint)
            nextLeftTransform.PreMultiply()
            nextLeftTransform.Concatenate(stepPoseLeft)

            if self.leadingFootByUser == 'Right':
                footsteps.append(Footstep(nextRightTransform,True))
                footsteps.append(Footstep(nextLeftTransform,False))
            else:
                footsteps.append(Footstep(nextLeftTransform,False))
                footsteps.append(Footstep(nextRightTransform,True))

        if (removeFirstLeftStep is True):
            if (standingFootName is self.ikPlanner.rightFootLink ):
                footsteps = footsteps[1:]
                print "Removing the first left step"

        return footsteps


    def sendFootstepPlanRequest(self, footsteps, nextDoubleSupportPose):
        goalSteps = []

        for i, footstep in enumerate(footsteps):
            step_t = footstep.transform

            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot =  footstep.is_right_foot # flist[i,6] # is_right_foot
            step.params = self.footstepsDriver.getDefaultStepParams()

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

        lcmUtils.publish('FOOTSTEP_PLAN_REQUEST', request)

    def executePlan(self):
        self.footstepsDriver.commitFootstepPlan(self.footstepsDriver.lastFootstepPlan)

class ManualWalkingTaskPanel(TaskUserPanel):

    def __init__(self, manualWalkingDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.manualWalkingDemo = manualWalkingDemo

        self.addDefaultProperties()
        self.addTasks()
        self.addButtons()


    def addButtons(self):

        self.addManualButton('EXECUTE Plan', self.manualWalkingDemo.executePlan)

    def addDefaultProperties(self):
        self.params.addProperty('Leading Foot', 0, attributes=om.PropertyAttributes(enumNames=['Left','Right']))
        self.params.addProperty('Num Steps', 3, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Forward Step', 0.35, attributes=om.PropertyAttributes(decimals=2, minimum=-0.6, maximum=0.6, singleStep=0.01))
        self.params.addProperty('Forward Balance Step', 0.05, attributes=om.PropertyAttributes(decimals=2, minimum=-0.6, maximum=0.6, singleStep=0.01))
        self.params.addProperty('Step Width', 0.25, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.3, singleStep=0.01))
        self.params.addProperty('Lateral Shift', 0.0, attributes=om.PropertyAttributes(decimals=2, minimum=-0.4, maximum=0.4, singleStep=0.01))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.manualWalkingDemo.testManualWalking()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.manualWalkingDemo.leadingFootByUser = 'Left'
        else:
            self.manualWalkingDemo.leadingFootByUser = 'Right'
    
        self.manualWalkingDemo.numSteps = self.params.getProperty('Num Steps')
        self.manualWalkingDemo.forwardStep = self.params.getProperty('Forward Step')
        self.manualWalkingDemo.forwardBalance = self.params.getProperty('Forward Balance Step')
        self.manualWalkingDemo.stepWidth = self.params.getProperty('Step Width')
        self.manualWalkingDemo.lateralShift = self.params.getProperty('Lateral Shift')

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        mw = self.manualWalkingDemo

        ###############
        # add the tasks

        # plan walking
        load = self.taskTree.addGroup('Placing Footsteps')
        addFunc(functools.partial(mw.testManualWalking), 'place footsteps', parent=load)

