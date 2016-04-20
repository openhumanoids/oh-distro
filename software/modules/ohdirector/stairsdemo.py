import os
import operator
import functools
import vtkNumpy
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
from director.debugVis import DebugData
from director.lcmframe import positionMessageFromFrame

import drc as lcmdrc

class BlockTop():
    def __init__(self, cornerTransform, rectDepth, rectWidth, rectArea):
        self.cornerTransform = cornerTransform # location of far right corner
        self.rectDepth = rectDepth # length of face away from robot
        self.rectWidth = rectWidth # length of face perpendicular to robot's toes
        self.rectArea = rectArea


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
        self.onSyncProperties = False

        self.plans = []
        self.blocksList = []

        # clusters and blocks
        self.groundWidthThresh = 1.00
        self.groundDepthThresh = 1.10

        # Footsteps placement options
        self.isLeadingFootRight = True


    def testStairs(self, leadFoot=None):
        if (leadFoot is None):
            if self.isLeadingFootRight:
                leadFoot=self.ikPlanner.rightFootLink #'rightFoot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'leftFoot'

        polyData = segmentation.getCurrentRevolutionData()
        startPose = self.getEstimatedRobotStatePose()

        self.footstepsPlacement(polyData, leadFoot, nextDoubleSupportPose = startPose)


    def footstepsPlacement(self, polyData, standingFootName, nextDoubleSupportPose = None):
        standingFootFrame = self.robotStateModel.getLinkFrame(standingFootName)
        vis.updateFrame(standingFootFrame, standingFootName, parent='cont debug', visible=False)

        # Prevent segmentation at each property update
        if not self.onSyncProperties:
            # Step 1: filter the data down to a box in front of the robot:
            polyData = self.getRecedingTerrainRegion(polyData, footstepsdriver.FootstepsDriver.getFeetMidPoint(self.robotStateModel))

            # Step 2: find all the surfaces in front of the robot (about 0.75sec)
            clusters = segmentation.findHorizontalSurfaces(polyData, removeGroundFirst=False, normalEstimationSearchRadius=0.05,
                                                           clusterTolerance=0.045, distanceToPlaneThreshold=0.0025, normalsDotUpRange=[0.95, 1.0])
            if clusters is None:
                print "No cluster found, stop walking now!"
                return

            # Step 3: find the corners of the minimum bounding rectangles
            blocks = self.extractBlocksFromSurfaces(clusters, standingFootFrame)

            self.blocksList = blocks
        else:
            self.onSyncProperties = False

        # Step 4: Footsteps placement
        footsteps = self.placeStepsOnBlocks(self.blocksList)

        assert len(footsteps) > 0

        print 'got %d footsteps' % len(footsteps)

        # Step 5: Send request to planner. It replies with complete footsteps plan message.
        self.sendFootstepPlanRequest(footsteps, nextDoubleSupportPose)

    def placeStepsOnBlocks(self, blocks):
        contact_pts_left, contact_pts_right = self.footstepsDriver.getContactPts()

        footsteps = []
        print 'got %d blocks' % len(blocks)
        for i, block in enumerate(blocks):
            blockBegin = transformUtils.copyFrame(block.cornerTransform)
            blockBegin.PreMultiply()
            cornerToCenterTransform = transformUtils.frameFromPositionAndRPY([-block.rectDepth,block.rectWidth/2,0.0], [0,0,0])
            blockBegin.Concatenate(cornerToCenterTransform)
            vis.updateFrame(blockBegin, 'block begin %d' % i , parent='block begins', scale=0.2, visible=True)

            # TO_DO find a better way to get the vertical translation of footsteps reference frame
            nextRightTransform = transformUtils.copyFrame(blockBegin)
            rightTransform = transformUtils.frameFromPositionAndRPY([self.forwardStepRight,-self.stepWidth/2,-contact_pts_left[0][2]], [0,0,0])
            nextRightTransform.PreMultiply()
            nextRightTransform.Concatenate(rightTransform)
            vis.updateFrame(transformUtils.copyFrame(nextRightTransform), 'right transform %d' % i , parent='block begins', scale=0.2, visible=True)

            nextLeftTransform = transformUtils.copyFrame(blockBegin)
            leftTransform = transformUtils.frameFromPositionAndRPY([self.forwardStepLeft,self.stepWidth/2,-contact_pts_left[0][2]], [0,0,0])
            nextLeftTransform.PreMultiply()
            nextLeftTransform.Concatenate(leftTransform)
            vis.updateFrame(transformUtils.copyFrame(nextLeftTransform), 'left transform %d' % i , parent='block begins', scale=0.2, visible=True)

            if self.isLeadingFootRight:
                footsteps.append(Footstep(nextRightTransform,True))
                footsteps.append(Footstep(nextLeftTransform,False))
            else:
                footsteps.append(Footstep(nextLeftTransform,False))
                footsteps.append(Footstep(nextRightTransform,True))

        return footsteps

    def getRecedingTerrainRegion(self, polyData, linkFrame):
        ''' Find the point cloud in front of the foot frame'''
        points = vtkNumpy.getNumpyFromVtk(polyData, 'Points')

        viewOrigin = linkFrame.TransformPoint([0.0, 0.0, 0.0])
        viewX = linkFrame.TransformVector([1.0, 0.0, 0.0])
        viewY = linkFrame.TransformVector([0.0, 1.0, 0.0])
        viewZ = linkFrame.TransformVector([0.0, 0.0, 1.0])
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewX, origin=viewOrigin, resultArrayName='distance_along_foot_x')
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewY, origin=viewOrigin, resultArrayName='distance_along_foot_y')
        polyData = segmentation.labelPointDistanceAlongAxis(polyData, viewZ, origin=viewOrigin, resultArrayName='distance_along_foot_z')

        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_x', [0.12, self.groundDepthThresh])
        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_y', [-(self.groundWidthThresh/2), self.groundWidthThresh/2])
        polyData = segmentation.thresholdPoints(polyData, 'distance_along_foot_z', [-(self.groundWidthThresh/2), self.groundWidthThresh/2])

        vis.updatePolyData( polyData, 'walking snapshot trimmed', parent='cont debug', visible=True)
        return polyData

    def extractBlocksFromSurfaces(self, clusters, linkFrame):
        ''' find the corners of the minimum bounding rectangles '''
        om.removeFromObjectModel(om.findObjectByName('block corners'))
        om.getOrCreateContainer('block corners',om.getOrCreateContainer('stairs'))

        # get the rectangles from the clusters:
        blocks = []
        for i, cluster in enumerate(clusters):
                cornerTransform, rectDepth, rectWidth, rectArea = segmentation.findMinimumBoundingRectangle( cluster, linkFrame )
                #print 'min bounding rect:', rectDepth, rectWidth, rectArea, cornerTransform.GetPosition()

                block = BlockTop(cornerTransform, rectDepth, rectWidth, rectArea)
                blocks.append(block)

        # filter out blocks that are too big or small
        blocksGood = []
        groundPlane = None

        stepWidthMaxThresh = 0.65
        stepDepthMaxThresh = 0.65
        stepWidthMinThresh = 0.35
        stepDepthMinThresh = 0.30

        for i, block in enumerate(blocks):
            if ((block.rectWidth<stepWidthMaxThresh) and (block.rectDepth<stepDepthMaxThresh)
                and (block.rectWidth>stepWidthMinThresh) and (block.rectDepth>stepDepthMinThresh)):
                blocksGood.append(block)
            else:
                groundPlane = block
        blocks = blocksGood

        # order by distance from robot's foot
        for i, block in enumerate(blocks):
            block.distToRobot = np.linalg.norm(np.array(linkFrame.GetPosition()) - np.array(block.cornerTransform.GetPosition()))
        blocks.sort(key=operator.attrgetter('distToRobot'))

        # Visualization:
        # draw blocks including the ground plane:
        om.removeFromObjectModel(om.findObjectByName('blocks'))
        blocksFolder=om.getOrCreateContainer('blocks',om.getOrCreateContainer('stairs'))
        for i, block in enumerate(blocks):
            vis.updateFrame(block.cornerTransform, 'block corners %d' % i , parent='block corners', scale=0.2, visible=True)

            blockCenter = transformUtils.copyFrame(block.cornerTransform)
            blockCenter.PreMultiply()
            cornerToCenterTransform = transformUtils.frameFromPositionAndRPY([-block.rectDepth/2,block.rectWidth/2,0.0], [0,0,0])
            blockCenter.Concatenate(cornerToCenterTransform)

            d = DebugData()
            d.addCube([ block.rectDepth, block.rectWidth,0.005],[0,0,0])
            obj = vis.showPolyData(d.getPolyData(),'block %d' % i, color=[1,0,1],parent=blocksFolder)
            obj.actor.SetUserTransform(blockCenter)

        if (groundPlane is not None):
            vis.updateFrame(groundPlane.cornerTransform, 'ground plane', parent='block corners', scale=0.2, visible=True)

            blockCenter = transformUtils.copyFrame(groundPlane.cornerTransform)
            blockCenter.PreMultiply()
            cornerToCenterTransform = transformUtils.frameFromPositionAndRPY([-groundPlane.rectDepth/2,groundPlane.rectWidth/2,0.0], [0,0,0])
            blockCenter.Concatenate(cornerToCenterTransform)

            d = DebugData()
            d.addCube([ groundPlane.rectDepth, groundPlane.rectWidth,0.005],[0,0,0])
            obj = vis.showPolyData(d.getPolyData(),'ground plane', color=[1,1,0],alpha=0.1, parent=blocksFolder)
            obj.actor.SetUserTransform(blockCenter)

        return blocks

    def sendFootstepPlanRequest(self, footsteps, nextDoubleSupportPose):
        goalSteps = []

        for i, footstep in enumerate(footsteps):
            step_t = footstep.transform

            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot = footstep.is_right_foot
            step.fixed_z = True
            step.is_in_contact = True
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
        request.params.planning_mode = lcmdrc.footstep_plan_params_t.MODE_AUTO
        request.params.map_mode = lcmdrc.footstep_plan_params_t.FOOT_PLANE
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

        self.autoQuery = True

    def addButtons(self):
        self.addManualButton('Load Scenario', self.stairsDemo.loadSDFFileAndRunSim)
        self.addManualButton('Footsteps Plan', self.stairsDemo.testStairs)
        self.addManualButton('EXECUTE Plan', self.stairsDemo.executePlan)

    def setDefaults(self, makeQuery = True):
        self.autoQuery = False
        # Footsteps placement options (in ui)
        self.params.setProperty('Leading Foot', 1) # Right
        self.params.setProperty('Forward Step Right', 0.10)
        self.params.setProperty('Forward Step Left', 0.18)
        self.params.setProperty('Step Width', 0.25)
        # IHMC params
        self.params.setProperty('IHMC Transfer Time', 1.0)
        self.params.setProperty('IHMC Swing Time', 1.0)

        self._syncProperties()
        if (makeQuery):
            self.stairsDemo.testStairs()
        self.autoQuery = True

    def addDefaultProperties(self):
        self.params.addProperty('Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Left','Right']))
        self.params.addProperty('Forward Step Right', 0.10, attributes=om.PropertyAttributes(decimals=2, minimum=0.05, maximum=0.25, singleStep=0.01))
        self.params.addProperty('Forward Step Left', 0.18, attributes=om.PropertyAttributes(decimals=2, minimum=0.05, maximum=0.25, singleStep=0.01))
        self.params.addProperty('Step Width', 0.25, attributes=om.PropertyAttributes(decimals=2, minimum=0.15, maximum=0.6, singleStep=0.01))
        self.params.addProperty('IHMC Transfer Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.25, maximum=2.0, singleStep=0.01))
        self.params.addProperty('IHMC Swing Time', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.6, maximum=1.5, singleStep=0.01))

        # Set the dafault values in ui
        self.setDefaults(False)

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        if (self.autoQuery):
            self.stairsDemo.onSyncProperties = True
            self.stairsDemo.testStairs()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.stairsDemo.isLeadingFootRight = False
        else:
            self.stairsDemo.isLeadingFootRight = True

        self.stairsDemo.forwardStepRight = self.params.getProperty('Forward Step Right')
        self.stairsDemo.forwardStepLeft = self.params.getProperty('Forward Step Left')
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
            self.folder = prevFolder

        sd = self.stairsDemo

        self.taskTree.removeAllTasks()

        # add tasks
        # prep
        addFolder('prep')
        addTask(rt.SetNeckPitch(name='set neck position', angle=50))
        addManipTask('move hands down', sd.planArmsDown, userPrompt=True)

