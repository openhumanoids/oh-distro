import os
import copy
import math
import functools
import numpy as np

import drcargs

from director import transformUtils

from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import robotstate
from director import segmentation
from director import planplayback
from director.pointpicker import PointPicker
from director import vtkAll as vtk
from director.simpletimer import SimpleTimer
from director import affordanceupdater
from director import sceneloader

from director.debugVis import DebugData
from director import affordanceitems
from director import ikplanner
from director import vtkNumpy
from numpy import array
from director.uuidutil import newUUID
from director import lcmUtils
import ioUtils
import drc as lcmdrc

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt


class TableboxDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner,
                 lhandDriver, rhandDriver, view, sensorJointController):
        # TODO self.planPlaybackFunction = planPlaybackFunction
        self.robotStateModel = robotStateModel
        self.playbackRobotModel = playbackRobotModel
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.footstepPlanner = footstepPlanner
        # TODO self.atlasDriver = atlasDriver
        self.lhandDriver = lhandDriver
        self.rhandDriver = rhandDriver
        # TODO self.multisenseDriver = multisenseDriver
        self.sensorJointController = sensorJointController
        self.view = view
        self.affordanceManager = segmentation.affordanceManager

        # live operation flags:
        self.useFootstepPlanner = True
        self.visOnly = False
        self.planFromCurrentRobotState = True
        extraModels = [self.playbackRobotModel]
        self.affordanceUpdater  = affordanceupdater.AffordanceGraspUpdater(self.robotStateModel, self.ikPlanner, extraModels)

        self.affordanceManager.setAffordanceUpdater(self.affordanceUpdater)
        self.optionalUserPromptEnabled = True
        self.requiredUserPromptEnabled = True

        self.plans = []
        self.clusterObjects = []
        self.frameSyncs = {}

        self.tableData = None

        # top level switch between BDI or IHMC (locked base) and MIT (moving base and back)
        self.lockBack = True
        self.lockBase = True
        self.planner = None

        self.constraintSet = []

        self.picker = None


    def planPlaybackFunction(plans):
        planPlayback.stopAnimation()
        playbackRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, playbackJointController)

    def addPlan(self, plan):
        self.plans.append(plan)


    ### Table and Bin Focused Functions
    def userFitTable(self):
        self.tableData = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=self.onSegmentTable)
        self.picker.start()


    def waitForTableFit(self):
        while not self.tableData:
            yield


    def getInputPointCloud(self):
        polyData = segmentation.getCurrentRevolutionData()
        if polyData is None:
            obj = om.findObjectByName('scene')
            if obj:
                polyData = obj.polyData

        return polyData

    def onSegmentTable(self, p1, p2):
        print p1
        print p2
        if self.picker is not None:
            self.picker.stop()
            om.removeFromObjectModel(self.picker.annotationObj)
            self.picker = None

        tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)

        pose = transformUtils.poseFromTransform(tableData.frame)
        desc = dict(classname='MeshAffordanceItem', Name='table', Color=[0,1,0], pose=pose)
        aff = self.affordanceManager.newAffordanceFromDescription(desc)
        aff.setPolyData(tableData.mesh)

        self.tableData = tableData

        tableBox = vis.showPolyData(tableData.box, 'table box', parent=aff, color=[0,1,0], visible=False)
        tableBox.actor.SetUserTransform(tableData.frame)

        # how far back to stand - from the middle of the table
        # -0.6 is too far. reduced 0.5 was too low. now trying -0.55
        relativeStance = transformUtils.frameFromPositionAndRPY([-0.55, 0, 0],[0,0,0])
        self.computeTableStanceFrame(relativeStance)

    def computeTableStanceFrame(self, relativeStance):
        tableTransform = om.findObjectByName('table').getChildFrame().transform
        zGround = 0.0
        tableHeight = tableTransform.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(relativeStance.GetPosition()[0], relativeStance.GetPosition()[1], -tableHeight)
        t.Concatenate(tableTransform)
        vis.showFrame(t, 'table stance frame', parent=om.findObjectByName('table'), scale=0.2)


    # TODO: deprecate this function: (to end of section):
    def moveRobotToTableStanceFrame(self):
        self.teleportRobotToStanceFrame(om.findObjectByName('table stance frame').transform)

    ### End Object Focused Functions ###############################################################
    ### Planning Functions ########################################################################

    def planFootsteps(self, goalFrame):
        startPose = self.getPlanningStartPose()
        request = self.footstepPlanner.constructFootstepPlanRequest(startPose, goalFrame)
        self.footstepPlan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        plan = self.footstepPlanner.sendWalkingPlanRequest(self.footstepPlan, startPose, waitForResponse=True)
        self.addPlan(plan)

    def planWalkToStance(self, stanceTransform):
        if self.useFootstepPlanner:
            self.planFootsteps(stanceTransform)
            self.planWalking()
        else:
            self.teleportRobotToStanceFrame(stanceTransform)

    def planPostureFromDatabase(self, groupName, postureName, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, groupName, postureName, side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        # TODO: integrate this function with the ones below

    def planPreGrasp(self, side='left'):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'arm up pregrasp', side=side)
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        

    ### End Planning Functions ####################################################################
    ########## Glue Functions #####################################################################
    def teleportRobotToStanceFrame(self, frame):
        stancePosition = frame.GetPosition()
        stanceOrientation = frame.GetOrientation()

        q = self.sensorJointController.q.copy()
        q[:2] = [stancePosition[0], stancePosition[1]]
        q[5] = math.radians(stanceOrientation[2])
        self.sensorJointController.setPose('EST_ROBOT_STATE', q)


    def getHandDriver(self, side):
        assert side in ('left', 'right')
        return self.lhandDriver if side == 'left' else self.rhandDriver

    def openHand(self, side):
        #self.getHandDriver(side).sendOpen()
        self.getHandDriver(side).sendCustom(0.0, 100.0, 100.0, 0)

    def closeHand(self, side):
        self.getHandDriver(side).sendCustom(100.0, 100.0, 100.0, 0)

    # TODO: re-enable
    #def sendNeckPitchLookDown(self):
    #    self.multisenseDriver.setNeckPitch(40)

    # TODO: re-enable
    #def sendNeckPitchLookForward(self):
    #    self.multisenseDriver.setNeckPitch(15)

    # TODO: re-enable
    #def waitForAtlasBehaviorAsync(self, behaviorName):
    #    assert behaviorName in self.atlasDriver.getBehaviorMap().values()
    #    while self.atlasDriver.getCurrentBehaviorName() != behaviorName:
    #        yield

    def printAsync(self, s):
        yield
        print s

    def optionalUserPrompt(self, message):
        if not self.optionalUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def requiredUserPrompt(self, message):
        if not self.requiredUserPromptEnabled:
            return

        yield
        result = raw_input(message)
        if result != 'y':
            raise Exception('user abort.')

    def delay(self, delayTimeInSeconds):
        yield
        t = SimpleTimer()
        while t.elapsed() < delayTimeInSeconds:
            yield

    # TODO: re-enable
    #def waitForCleanLidarSweepAsync(self):
    #    currentRevolution = self.multisenseDriver.displayedRevolution
    #    desiredRevolution = currentRevolution + 2
    #    while self.multisenseDriver.displayedRevolution < desiredRevolution:
    #        yield

    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def cleanupFootstepPlans(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))
        om.removeFromObjectModel(om.findObjectByName('footstep plan'))
        self.footstepPlan = None

    def playSequenceNominal(self):
        assert None not in self.plans
        self.planPlaybackFunction(self.plans)

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def commitFootstepPlan(self):
        self.footstepPlanner.commitFootstepPlan(self.footstepPlan)

    def waitForPlanExecution(self, plan):
        planElapsedTime = planplayback.PlanPlayback.getPlanElapsedTime(plan)
        return self.delay(planElapsedTime + 1.0)


    def animateLastPlan(self):
        plan = self.plans[-1]
        if not self.visOnly:
            self.commitManipPlan()

        return self.waitForPlanExecution(plan)

    def onRobotModelChanged(self, model):

        for linkName in self.frameSyncs.keys():
            t = self.playbackRobotModel.getLinkFrame(linkName)
            vis.updateFrame(t, '%s frame' % linkName, scale=0.2, visible=False, parent='planning')


    #################################
    def loadTestPointCloud(self):
        filename = os.environ['DRC_BASE'] +  '/../drc-testing-data/tabletop/table-box-uoe.vtp'
        polyData = ioUtils.readPolyData( filename )
        vis.showPolyData(polyData,'scene')


        stanceFrame = transformUtils.frameFromPositionAndRPY([0.0, 0, 0.0], [0,0,-135])
        self.teleportRobotToStanceFrame(stanceFrame)


    def spawnBlockAffordance(self):
        boxLength = 0.3
        boxWidth = 0.25
        boxHeight = 0.3

        boxFrame = self.footstepPlanner.getFeetMidPoint(self.robotStateModel)
        boxFrame.PreMultiply()
        boxFrame.Translate([0.49, 0.0, 1.02])
        #vis.updateFrame(boxFrame, 'boxFrame')

        xAxis,yAxis, zAxis = transformUtils.getAxesFromTransform(boxFrame)
        segmentation.createBlockAffordance(boxFrame.GetPosition(), xAxis,yAxis, zAxis, boxLength, boxWidth, boxHeight, 'box', parent='affordances')


    def planSimpleBoxGrasp(self):
        ikplanner.getIkOptions().setProperty('Use pointwise', False)

        deliveryAffordance = om.findObjectByName('box')
        boxFrame = deliveryAffordance.getChildFrame().transform
        dim = deliveryAffordance.getProperty('Dimensions')
        palmSeperation = (dim[1] - 0.14)/2.0
        print palmSeperation


        leftFrame = transformUtils.frameFromPositionAndRPY([0.0, palmSeperation, 0.0], [90,90,0])
        leftFrame = transformUtils.concatenateTransforms([leftFrame, boxFrame])
        vis.updateFrame(leftFrame, 'reach left')

        rightFrame = transformUtils.frameFromPositionAndRPY([0.0, -palmSeperation, 0.0], [0,-90,-90])
        rightFrame = transformUtils.concatenateTransforms([rightFrame, boxFrame])
        vis.updateFrame(rightFrame, 'reach right')

        startPose = self.getPlanningStartPose()
        self.constraintSet = self.ikPlanner.planEndEffectorGoal(startPose, 'left', leftFrame, lockBase=self.lockBase, lockBack=self.lockBack, lockArm=False)

        startPoseName = 'reach_start'
        self.constraintSet = self.ikPlanner.newReachGoal(startPoseName, 'right', rightFrame, self.constraintSet.constraints, None)

        self.constraintSet.runIk()
        print 'planning bihanded reach'
        plan = self.constraintSet.runIkTraj()
        self.addPlan(plan)

        ikplanner.getIkOptions().setProperty('Use pointwise', True)


    def planWalkToTable(self):
        self.planWalkToStance(om.findObjectByName('table stance frame').transform)


    def planArmsSpread(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Table Box Pick', 'spread hands')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planArmsRaise(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Table Box Pick', 'hands goalie')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planArmsDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown incl back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


'''
Tableboxdemo Image Fit for live-stream of webcam
'''
class TableImageFitter(ImageBasedAffordanceFit):

    def __init__(self, tableboxDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.tableboxDemo = tableboxDemo

    def fit(self, polyData, points):
        pass

'''
Tablebox Task Panel
'''
class TableboxTaskPanel(TaskUserPanel):

    def __init__(self, tableboxDemo):

        TaskUserPanel.__init__(self, windowTitle='Table Task')

        self.tableboxDemo = tableboxDemo
        self.tableboxDemo.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.fitter = TableImageFitter(self.tableboxDemo)
        self.initImageView(self.fitter.imageView, activateAffordanceUpdater=False)

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Load Test Cloud', self.tableboxDemo.loadTestPointCloud)
        self.addManualSpacer()        

        p1 = np.array([-0.58354658, -0.98459125, 0.75729603])
        p2 = np.array([-0.40979841, -0.76145965,  0.73299527])

        self.addManualButton('User Table', functools.partial(self.tableboxDemo.onSegmentTable, p1, p2) )        
        self.addManualButton('Move to Stance',self.tableboxDemo.moveRobotToTableStanceFrame)
        self.addManualButton('Spread Arms',self.tableboxDemo.planArmsSpread)


    def addDefaultProperties(self):
        self.params.addProperty('Base', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Back', 1,
                                    attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))

        self.params.addProperty('Scene', 0, attributes=om.PropertyAttributes(enumNames=['Objects on table','Object below table','Object through slot','Object at depth','Objects on table (fit)']))

        # Init values as above
        self.tableboxDemo.lockBase = self.getLockBase()
        self.tableboxDemo.lockBack = self.getLockBack()

    def getLockBase(self):
        return True if self.params.getPropertyEnumValue('Base') == 'Fixed' else False

    def getLockBack(self):
        return True if self.params.getPropertyEnumValue('Back') == 'Fixed' else False

    def getHandEngaged(self):
        return self.params.getProperty('Hand Engaged (Powered)')

    def getPlanner(self):
        return self.params.getPropertyEnumValue('Planner') if self.params.hasProperty('Planner') else None

    def onPropertyChanged(self, propertySet, propertyName):
        propertyName = str(propertyName)

        if propertyName == 'Base':
            self.tableboxDemo.lockBase = self.getLockBase()

        elif propertyName == 'Back':
            self.tableboxDemo.lockBack = self.getLockBack()

    def pickupMoreObjects(self):
        if len(self.tableboxDemo.clusterObjects) > 0: # There is still sth on the table, let's do it again!
            print "There are more objects on the table, going at it again!"
            self.taskTree.selectTaskByName('reach')
            self.onPause()
            self.onContinue()
        else:
            print "Table clear, sir!"

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


        def addManipulation(func, name, parent=None, confirm=False):
            group = self.taskTree.addGroup(name, parent=parent)
            addFunc(func, name='plan motion', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(v.commitManipPlan, name='execute manip plan', parent=group)

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan motion', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.tableboxDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        v = self.tableboxDemo

        self.taskTree.removeAllTasks()


        ###############
        # find the table
        addFolder('approach')
        addManipTask('move hands down', v.planArmsDown, userPrompt=True)
        addFunc('activate table fit', self.tableboxDemo.userFitTable)
        addTask(rt.UserPromptTask(name='approve table fit', message='Please approve the table fit.'))

        # walk to table
        addFolder('walk')
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='table stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        #- raise arms
        addFolder('prep')
        addManipTask('spread arms', v.planArmsSpread, userPrompt=True)
        addManipTask('raise arms', v.planArmsRaise, userPrompt=True)

        #- fit box
        #- grasp box
        addFolder('grasp')
        addFunc('spawn block affordance', v.spawnBlockAffordance)
        addManipTask('Box grasp', v.planSimpleBoxGrasp, userPrompt=True)

        #- lift box
        #- walk backwards