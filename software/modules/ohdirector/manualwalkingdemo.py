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
        self.forwardBalance = 0.05
        self.stepWidth = 0.25
        self.lateralShift = 0.0

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
        
        # Place footsteps an the ground manually (no planning)
        # The number of footsteps, width and forward distance between steps are given by user.
        footsteps = []

        startFeetMidPointPos = startFeetMidPoint.GetPosition()

        contactPtsLeft, contactPtsRight = self.footstepsDriver.getContactPts()
        contactPtsMid = np.mean(contactPtsRight, axis=0) # mid point on foot relative to foot frame

        j = 1
        for i in range(self.numSteps):   
            if i % 2 == 0:
                forward = self.forwardStep*j - contactPtsMid[0]
                if not self.isLeadingFootRight:
                    width = self.stepWidth / 2 + self.lateralShift * j
                else:
                    width = -self.stepWidth / 2 + self.lateralShift * j

                stepPose = transformUtils.frameFromPositionAndRPY([forward, width, startFeetMidPointPos[2]], [0,0,0])

                nextTransform = transformUtils.copyFrame(startFeetMidPoint)
                nextTransform.PreMultiply()
                nextTransform.Concatenate(stepPose)
               
                footsteps.append(Footstep(nextTransform,self.isLeadingFootRight))
            else:
                forward = self.forwardStep * j - contactPtsMid[0] + self.forwardBalance
                if not self.isLeadingFootRight:
                    width = -self.stepWidth / 2 + self.lateralShift * j
                else:
                    width = self.stepWidth / 2 + self.lateralShift * j

                stepPose = transformUtils.frameFromPositionAndRPY([forward, width, startFeetMidPointPos[2]], [0,0,0])

                nextTransform = transformUtils.copyFrame(startFeetMidPoint)
                nextTransform.PreMultiply()
                nextTransform.Concatenate(stepPose)

                isLeadingFootLeft = not(self.isLeadingFootRight) 
                footsteps.append(Footstep(nextTransform,isLeadingFootLeft))
                j = j+1

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
        self.params.addProperty('Num Steps', 4, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
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
            self.manualWalkingDemo.isLeadingFootRight = False
        else:
            self.manualWalkingDemo.isLeadingFootRight = True
    
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

