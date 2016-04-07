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

from director.roboturdf import HandFactory
from numpy import linalg as npla
from director import ik
import director.applogic as app


class TableplanningDemo(object):

    def __init__(self, robotStateModel, playbackRobotModel, ikPlanner, manipPlanner, footstepPlanner,
                 lhandDriver, rhandDriver, view, sensorJointController, teleopRobotModel, teleopJointController, footstepsDriver):
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
        
        self.teleopRobotModel = teleopRobotModel
        self.teleopJointController = teleopJointController
        self.footstepsDriver = footstepsDriver
        self.feetConstraint = 'Sliding'
        self.reachingHand = 'right'
        self.ikPlanner.planningMode = 'drake'
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
    
    def userReFitTable(self):
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        om.removeFromObjectModel(om.findObjectByName('cont debug'))
        om.removeFromObjectModel(om.findObjectByName('affordances'))
        self.userFitTable()
        
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

    def onEndPosePlanning(self):
        self.feetConstraint = 'Sliding'
        self.placeHandModel()
        
    def onMotionPlanning(self):
        self.feetConstraint = 'Fixed'
        self.placeHandModel()            
        self.runIK()
        startPoseName = 'reach_start'
        startPose = np.array(self.sensorJointController.q)
        self.ikPlanner.addPose(startPose, startPoseName)
        newPlan = self.constraintSet.runIkTraj()
        self.addPlan(newPlan)
        self.showPlan(newPlan)

    def placeHandModel(self):
        if not om.findObjectByName('Final Pose End Effector'):
            side = self.reachingHand
            handFrame = self.footstepPlanner.getFeetMidPoint(self.robotStateModel)
            handFrame.PreMultiply()
            handFrame.Translate([0.49, 0.0, 1.02])
            if side == 'left':
                rotation = [0, 90, -90]
            else:
                rotation = [0, -90, -90]
            handFrame.Concatenate(transformUtils.frameFromPositionAndRPY([0,0,0], rotation))
            handFactory = HandFactory(self.robotStateModel)
            handFactory.placeHandModelWithTransform(handFrame, self.robotStateModel.views[0],
                                                    side, 'Final Pose End Effector', 'planning')
            handObj = om.findObjectByName('Final Pose End Effector frame')
            handObj.connectFrameModified(self.onHandModelModified)
        
    def onHandModelModified(self, frame):
        self.runIK()
        if self.feetConstraint == 'Sliding':
            stanceFrame = self.footstepsDriver.getFeetMidPoint(self.teleopRobotModel)
            vis.updateFrame(stanceFrame, 'table stance frame')
        
    def changeHand(self, handName):
        self.reachingHand = handName
        if om.findObjectByName('Final Pose End Effector'):
            om.removeFromObjectModel(om.findObjectByName('Final Pose End Effector'))
            self.placeHandModel()
            
    def runIK(self):
        self.createIKConstraints(baseConstraint='XYZ only', backConstraint='Fixed', feetConstraint=self.feetConstraint, reachHand=self.reachingHand)
        endPose, info = self.constraintSet.runIk()
        endPoseName = 'reach_end'
        self.ikPlanner.addPose(endPose, endPoseName)
        self.showPose(self.constraintSet.endPose)
        app.displaySnoptInfo(info)
        
    def showPose(self, pose):
        self.hidePlan()
        self.teleopJointController.setPose('reach_end', pose)
        self.showMPModel()
        
    def hideMPModel(self):
        self.teleopRobotModel.setProperty('Visible', False)
        
    def showMPModel(self):
        self.teleopRobotModel.setProperty('Visible', True)
        self.teleopRobotModel.setProperty('Alpha', 0.3)
        
    def showPlan(self, plan):
        self.hideMPModel()
        self.playbackRobotModel.setProperty('Visible', True)
        
    def hidePlan(self):
        self.playbackRobotModel.setProperty('Visible', False)
        
    def getGoalFrame(self, linkName):
        return om.findObjectByName('Final Pose End Effector frame')
        
    def createIKConstraints(self, baseConstraint, backConstraint, feetConstraint, reachHand):
        startPoseName = 'reach_start'
        startPose = np.array(self.getEstimatedRobotStatePose())
        self.ikPlanner.addPose(startPose, startPoseName)

        constraints = []
        constraints.append(self.ikPlanner.createQuasiStaticConstraint())
        constraints.append(self.ikPlanner.createLockedNeckPostureConstraint(startPoseName))
        
        # Get base constraint
        kneeJointLimits = drcargs.getDirectorConfig()['kneeJointLimits']
        if baseConstraint == 'Fixed':
            constraints.append(self.ikPlanner.createLockedBasePostureConstraint(startPoseName, lockLegs=False))
            self.ikPlanner.setBaseLocked(True)
        elif baseConstraint == 'XYZ only':
            constraints.append(self.ikPlanner.createXYZMovingBasePostureConstraint(startPoseName))
            constraints.append(self.ikPlanner.createKneePostureConstraint(kneeJointLimits))
        elif baseConstraint == 'Limited':
            constraints.append(self.ikPlanner.createMovingBaseSafeLimitsConstraint())
            constraints.append(self.ikPlanner.createKneePostureConstraint(kneeJointLimits))
            self.ikPlanner.setBaseLocked(False)
            
        # Get back constraint 
        if backConstraint == 'Fixed':
            constraints.append(self.ikPlanner.createLockedBackPostureConstraint(startPoseName))
            self.ikPlanner.setBackLocked(True)
        elif backConstraint == 'Limited':
            constraints.append(self.ikPlanner.createMovingBackLimitedPostureConstraint())
            self.ikPlanner.setBackLocked(False)
            
        # Get feet constraint
        if feetConstraint == 'Fixed':                
            constraints.append(self.ikPlanner.createFixedLinkConstraints(startPoseName, self.ikPlanner.leftFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
            constraints.append(self.ikPlanner.createFixedLinkConstraints(startPoseName, self.ikPlanner.rightFootLink, tspan=[0.0, 1.0], lowerBound=-0.0001*np.ones(3), upperBound=0.0001*np.ones(3), angleToleranceInDegrees=0.1))
        elif feetConstraint == 'Sliding':
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(startPoseName)[:2])
            constraints.extend(self.ikPlanner.createSlidingFootConstraints(startPoseName)[2:])
            
            # Ensure the end-pose's relative distance between two feet is the same as start pose
            [pos_left, quat_left] = transformUtils.poseFromTransform(self.robotStateModel.getLinkFrame(self.ikPlanner.leftFootLink))
            [pos_right, quat_right] = transformUtils.poseFromTransform(self.robotStateModel.getLinkFrame(self.ikPlanner.rightFootLink))
            dist = npla.norm(pos_left - pos_right)
            constraints.append(ik.PointToPointDistanceConstraint(bodyNameA=self.ikPlanner.leftFootLink, bodyNameB=self.ikPlanner.rightFootLink, lowerBound=np.array([dist - 0.0001]), upperBound=np.array([dist + 0.0001])))

        sides = []
        if reachHand == 'left':
            sides.append('left')
        elif reachHand == 'right':
            sides.append('right')
        elif reachHand == 'Both':
            sides.append('left')
            sides.append('right')

        if not 'left' in sides:
            self.ikPlanner.setArmLocked('left', True)
            constraints.append(self.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        if not 'right' in sides:
            self.ikPlanner.setArmLocked('right', True)
            constraints.append(self.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))
        
        for side in sides:        
            linkName = self.ikPlanner.getHandLink(side)
            graspToHand = self.ikPlanner.newPalmOffsetGraspToHandFrame(side, 0.0)
            graspToWorld = self.getGoalFrame(linkName)
            p, q = self.ikPlanner.createPositionOrientationGraspConstraints(side, graspToWorld, graspToHand)
            p.tspan = [1.0, 1.0]
            q.tspan = [1.0, 1.0]
            constraints.extend([p, q])
            constraints.append(self.ikPlanner.createActiveEndEffectorConstraint(linkName, self.ikPlanner.getPalmPoint(side)))
            
        self.constraintSet = ikplanner.ConstraintSet(self.ikPlanner, constraints, 'reach_end', startPoseName)
        

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
        curr = self.ikPlanner.planningMode
        self.ikPlanner.planningMode = 'drake'
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown incl back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)
        self.ikPlanner.planningMode = curr


'''
Tableboxdemo Image Fit for live-stream of webcam
'''
class TableImageFitter(ImageBasedAffordanceFit):

    def __init__(self, tableplanningDemo):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.tableplanningDemo = tableplanningDemo

    def fit(self, polyData, points):
        pass

'''
Tablebox Task Panel
'''
class TableplanningTaskPanel(TaskUserPanel):

    def __init__(self, tableplanningDemo):

        TaskUserPanel.__init__(self, windowTitle='Table Task')

        self.tableplanningDemo = tableplanningDemo
        self.tableplanningDemo.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.fitter = TableImageFitter(self.tableplanningDemo)
        self.initImageView(self.fitter.imageView, activateAffordanceUpdater=False)

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Load Test Cloud', self.tableplanningDemo.loadTestPointCloud)
        self.addManualSpacer()        

        p1 = np.array([-0.58354658, -0.98459125, 0.75729603])
        p2 = np.array([-0.40979841, -0.76145965,  0.73299527])

        self.addManualButton('User Table', functools.partial(self.tableplanningDemo.onSegmentTable, p1, p2) )        
        self.addManualButton('Move to Stance',self.tableplanningDemo.moveRobotToTableStanceFrame)
        self.addManualButton('Spread Arms',self.tableplanningDemo.planArmsSpread)

    def addDefaultProperties(self):
        self.params.addProperty('Base', 1,
                                attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))
        self.params.addProperty('Back', 1,
                                    attributes=om.PropertyAttributes(enumNames=['Fixed', 'Free']))

        self.params.addProperty('Reaching Hand', 0,
                                    attributes=om.PropertyAttributes(enumNames=['right', 'left']))

        self.params.addProperty('PlanningMode', 0, attributes=om.PropertyAttributes(enumNames=['drake', 'exotica']))

        # Init values as above
        self.tableplanningDemo.lockBase = self.getLockBase()
        self.tableplanningDemo.lockBack = self.getLockBack()

    def getLockBase(self):
        return True if self.params.getPropertyEnumValue('Base') == 'Fixed' else False

    def getLockBack(self):
        return True if self.params.getPropertyEnumValue('Back') == 'Fixed' else False

    def getPlanningMode(self):
        return self.params.getPropertyEnumValue('PlanningMode')
    
    def getReachingHand(self):
        return self.params.getPropertyEnumValue('Reaching Hand')

    def getPlanner(self):
        return self.params.getPropertyEnumValue('Planner') if self.params.hasProperty('Planner') else None

    def onPropertyChanged(self, propertySet, propertyName):
        propertyName = str(propertyName)

        if propertyName == 'Base':
            self.tableplanningDemo.lockBase = self.getLockBase()

        elif propertyName == 'Back':
            self.tableplanningDemo.lockBack = self.getLockBack()
        elif propertyName == 'Reaching Hand':
            self.tableplanningDemo.changeHand(self.getReachingHand())
        elif propertyName == 'PlanningMode':
            self.tableplanningDemo.ikPlanner.planningMode = self.getPlanningMode()
            
    def pickupMoreObjects(self):
        if len(self.tableplanningDemo.clusterObjects) > 0: # There is still sth on the table, let's do it again!
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
            addFunc('execute manip plan', self.tableplanningDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        v = self.tableplanningDemo

        self.taskTree.removeAllTasks()

        ###############
        # preparation
        addFolder('prepare')
        addManipTask('move hands down', v.planArmsDown, userPrompt=True)
        addFunc('activate table fit', self.tableplanningDemo.userFitTable)
        addTask(rt.UserPromptTask(name='approve table fit', message='Please approve the table fit.'))
        
        
        # walk to table (end-pose planning)
        addFolder('end pose planning')
        addFunc('find stance pose', v.onEndPosePlanning)
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.SetNeckPitch(name='set neck position', angle=35))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='table stance frame footstep plan'))
        #addTask(rt.WaitForWalkExecution(name='wait for walking'))

        #- fit box
        #- grasp box
        addFolder('grasp')
        addManipTask('reach target', v.onMotionPlanning, userPrompt=True)

        #- lift box
        #- walk backwards