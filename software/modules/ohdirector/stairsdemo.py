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
from director import sceneloader
from director.lcmframe import positionMessageFromFrame

import drc as lcmdrc

class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot

class StairsDemo(object):
    def __init__(self, robotStateModel, footstepsDriver, robotStateJointController, ikPlanner, manipPlanner):
        self.footstepsDriver = footstepsDriver
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner

        # live operation flags:
        self.planFromCurrentRobotState = True

        self.plans = []

        # Manual footsteps placement options
        self.isLeadingFootRight = True
        self.numSteps = 6
        self.forwardStep = 0.35
        self.stepWidth = 0.25
        # IHMC params
        self.ihmcTransferTime = 1.0
        self.ihmcSwingTime = 1.0

    def testStairs(self, leadFoot=None):
        if (leadFoot is None):
            if self.isLeadingFootRight:
                leadFoot=self.ikPlanner.rightFootLink #'r_foot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'l_foot'

        feetMidPoint = self.footstepsDriver.getFeetMidPoint(self.robotStateModel)
        startPose = self.getEstimatedRobotStatePose()

        self.footstepsPlacement(leadFoot, startFeetMidPoint = feetMidPoint, nextDoubleSupportPose = startPose)


    def footstepsPlacement(self, standingFootName, startFeetMidPoint = None, nextDoubleSupportPose = None):
        # Step 1: filter the data down to a box in front of the robot:
        polyData = self.getRecedingTerrainRegion(polyData, footstepsdriver.FootstepsDriver.getFeetMidPoint(self.robotStateModel))

        # Step 2: find all the surfaces in front of the robot (about 0.75sec)
        clusters = segmentation.findHorizontalSurfaces(polyData, removeGroundFirst=False, normalEstimationSearchRadius=0.05,
                                                       clusterTolerance=0.025, distanceToPlaneThreshold=0.0025, normalsDotUpRange=[0.95, 1.0])
        if clusters is None:
            print "No cluster found, stop walking now!"
            return

        # Step 1: Footsteps placement
        '''footsteps = self.placeStepsManually(startFeetMidPoint)

        assert len(footsteps) > 0

        # Step 2: Send request to planner. It replies with complete footsteps plan message.
        self.sendFootstepPlanRequest(footsteps, nextDoubleSupportPose)'''


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

            if (i % 2 == 0 and not self.isLeadingFootRight) or (i % 2 != 0 and self.isLeadingFootRight):
                width = self.stepWidth / 2
            else:
                width = -self.stepWidth / 2

            stepPose = transformUtils.frameFromPositionAndRPY([forward, width, 0], [0,0,0])

            nextTransform = transformUtils.copyFrame(startFeetMidPoint)
            nextTransform.PreMultiply()
            nextTransform.Concatenate(stepPose)

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

    def loadSDFFileAndRunSim(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_simple_flagstones.sdf'  
        sc=sceneloader.SceneLoader()
        sc.loadSDF(filename)
        msg=lcmdrc.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)

    def addPlan(self, plan):
        self.plans.append(plan)

    def executePlan(self):
        self.footstepsDriver.commitFootstepPlan(self.footstepsDriver.lastFootstepPlan)

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def getEstimatedRobotStatePose(self):
        return self.robotStateJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def planArmsDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown incl back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

class StairsTaskPanel(TaskUserPanel):

    def __init__(self, stairsDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.stairsDemo = stairsDemo
        self.stairsDemo.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('Load Scenario', self.stairsDemo.loadSDFFileAndRunSim)
        self.addManualButton('Footsteps Plan', self.stairsDemo.testStairs)
        self.addManualButton('EXECUTE Plan', self.stairsDemo.executePlan)

    def addDefaultProperties(self):
        if self.stairsDemo.isLeadingFootRight:
            leadingFoot = 0
        else:
            leadingFoot = 1
        self.params.addProperty('Leading Foot', leadingFoot, attributes=om.PropertyAttributes(enumNames=['Right','Left']))
        self.params.addProperty('Num Steps', self.stairsDemo.numSteps, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Forward Step', self.stairsDemo.forwardStep, attributes=om.PropertyAttributes(decimals=2, minimum=-0.6, maximum=0.6, singleStep=0.01))
        self.params.addProperty('Step Width', self.stairsDemo.stepWidth, attributes=om.PropertyAttributes(decimals=2, minimum=0.15, maximum=0.6, singleStep=0.01))
        self.params.addProperty('IHMC Transfer Time', self.stairsDemo.ihmcTransferTime, attributes=om.PropertyAttributes(decimals=2, minimum=0.25, maximum=2.0, singleStep=0.01))
        self.params.addProperty('IHMC Swing Time', self.stairsDemo.ihmcSwingTime, attributes=om.PropertyAttributes(decimals=2, minimum=0.6, maximum=1.5, singleStep=0.01))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.stairsDemo.testStairs()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.stairsDemo.isLeadingFootRight = False
        else:
            self.stairsDemo.isLeadingFootRight = True
    
        self.stairsDemo.numSteps = self.params.getProperty('Num Steps')
        self.stairsDemo.forwardStep = self.params.getProperty('Forward Step')
        self.stairsDemo.stepWidth = self.params.getProperty('Step Width')
        self.stairsDemo.ihmcTransferTime = self.params.getProperty('IHMC Transfer Time')
        self.stairsDemo.ihmcSwingTime = self.params.getProperty('IHMC Swing Time')

    def addTasks(self):
        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):
            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan motion', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.stairsDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        sd = self.stairsDemo

        self.taskTree.removeAllTasks()

        # add tasks

        # prep
        addFolder('prep')
        addManipTask('move hands down', sd.planArmsDown, userPrompt=True)
        addTask(rt.SetNeckPitch(name='set neck position', angle=50))

