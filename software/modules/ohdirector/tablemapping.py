import os
import math
import functools

from director import transformUtils

from director import objectmodel as om
from director import visualization as vis
from director import robotstate
from director import segmentation
from director.pointpicker import PointPicker
from director import vtkAll as vtk
from director import sceneloader

from director import lcmUtils
from director.utime import getUtime

import drc as lcmdrc
from director import drcargs
import bot_core as lcmbotcore

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt

class SetNeckOrientation(rt.AsyncTask):

    @staticmethod
    def getDefaultProperties(properties):
        properties.addProperty('Angles', [0,0])

    def run(self):
        neckAngles = self.properties.getProperty('Angles')
        jointGroups = drcargs.getDirectorConfig()['teleopJointGroups']
        jointGroupNeck = filter(lambda group: group['name'] == 'Neck', jointGroups)
        if (len(jointGroupNeck) == 1):
            neckJoints = jointGroupNeck[0]['joints']
        else:
            return

        m = lcmbotcore.joint_angles_t()
        m.utime = getUtime()
        m.num_joints = 2
        m.joint_name = [ neckJoints[0], neckJoints[1] ] 
        m.joint_position = [ math.radians(neckAngles[0]), math.radians(neckAngles[1])]
        lcmUtils.publish('DESIRED_NECK_ANGLES', m)


class TableMapping(object):

    def __init__(self, robotStateModel, manipPlanner, view, ikPlanner, sensorJointController):
	self.robotStateModel = robotStateModel
	self.manipPlanner = manipPlanner
	self.ikPlanner = ikPlanner
        self.view = view
        self.sensorJointController = sensorJointController
        self.affordanceManager = segmentation.affordanceManager

	#live operation flags
        self.planFromCurrentRobotState = True
        self.plans = []

        self.tableData = None
	self.picker = None
	self.lowerNeckPitch = 0.;
	self.neckYaw = -0.;
	self.initialPose = None;

    #utilities
    def loadSDFFileAndRunSim(self):
        filename= os.environ['DRC_BASE'] + '/software/models/worlds/table_mapping.sdf'
        sc=sceneloader.SceneLoader()
        sc.loadSDF(filename)
        msg=lcmdrc.scs_api_command_t()
        msg.command="loadSDF "+filename+"\nsimulate"
        lcmUtils.publish('SCS_API_CONTROL', msg)

    def initRobotPose(self):
	robotState = self.getEstimatedRobotStatePose();
	self.initialPose = transformUtils.frameFromPositionAndRPY([robotState[0], robotState[1], robotState[2]],[0,0,0])


    def getEstimatedRobotStatePose(self):
        return self.sensorJointController.getPose('EST_ROBOT_STATE')

    def getInputPointCloud(self):
        polyData = segmentation.getCurrentRevolutionData()
        if polyData is None:
            obj = om.findObjectByName('scene')
            if obj:
                polyData = obj.polyData

        return polyData

    def getPlanningStartPose(self):
        if self.planFromCurrentRobotState:
            return self.getEstimatedRobotStatePose()
        else:
            if self.plans:
                return robotstate.convertStateMessageToDrakePose(self.plans[-1].plan[-1])
            else:
                return self.getEstimatedRobotStatePose()

    def setHeadPosition(self):
	neckOrientation = SetNeckOrientation(angles=[self.lowerNeckPitch, self.neckYaw])
	neckOrientation.run()
	
    #table detection
    def userFitTable(self, tableNumber):
        self.tableData = None
        self.picker = PointPicker(self.view, numberOfPoints=2, drawLines=True, callback=functools.partial(self.onSegmentTable, segmentTableNumber=tableNumber))
        self.picker.start()

    def waitForTableFit(self):
        while not self.tableData:
            yield

    def onSegmentTable(self, p1, p2, segmentTableNumber):
        print p1
        print p2
        if self.picker is not None:
            self.picker.stop()
            om.removeFromObjectModel(self.picker.annotationObj)
            self.picker = None

        tableData = segmentation.segmentTableEdge(self.getInputPointCloud(), p1, p2)

        pose = transformUtils.poseFromTransform(tableData.frame)

        desc = dict(classname='MeshAffordanceItem', Name='table ' + str(segmentTableNumber), Color=[0,1,0], pose=pose)
        aff = self.affordanceManager.newAffordanceFromDescription(desc)
        aff.setPolyData(tableData.mesh)

        self.tableData = tableData

        tableBox = vis.showPolyData(tableData.box, 'table box ' + str(segmentTableNumber), parent=aff, color=[0,1,0], visible=False)
        tableBox.actor.SetUserTransform(tableData.frame)

        relativeStance = transformUtils.frameFromPositionAndRPY([-0.6, 0, 0],[0,0,0])
        self.computeTableStanceFrame(relativeStance, str(segmentTableNumber))

    def computeTableStanceFrame(self, relativeStance, tableNumber):
        tableTransform = om.findObjectByName('table ' + str(tableNumber)).getChildFrame().transform
        zGround = 0.0
        tableHeight = tableTransform.GetPosition()[2] - zGround

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Translate(relativeStance.GetPosition()[0], relativeStance.GetPosition()[1], -tableHeight)
        t.Concatenate(tableTransform)
        vis.showFrame(t, 'table stance frame '+ str(tableNumber), parent=om.findObjectByName('table ' + str(tableNumber)), scale=0.2)

    def computeBackUpStanceFrame(self):
        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(self.initialPose)
        vis.showFrame(t, 'back up frame', scale=0.2)	        

    #manipulation planning - used for making the robot lower its arms at the start of the script
    def addPlan(self, plan):
        self.plans.append(plan)

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def planArmsDown(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown incl back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)


class TableImageFitter(ImageBasedAffordanceFit):

    def __init__(self, tableMapping):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.tableMapping = tableMapping

    def fit(self, polyData, points):
        pass


class TableTaskPanel(TaskUserPanel):

    def __init__(self, tableMapping):
        TaskUserPanel.__init__(self, windowTitle='Table Mapping Task')
        self.tableMapping = tableMapping
        self.tableMapping.planFromCurrentRobotState = True

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

        self.fitter = TableImageFitter(self.tableMapping)
        self.initImageView(self.fitter.imageView, activateAffordanceUpdater=False)

    def addDefaultProperties(self):
        self.params.addProperty('lowerNeckPitch', 0, attributes=om.PropertyAttributes(minimum=0, maximum=66.6, hidden=False, singleStep=0.01, decimals=3))
        self.params.addProperty('neckYaw', 0, attributes=om.PropertyAttributes(minimum=-60, maximum=60, hidden=False, singleStep=0.01, decimals=3))

    def onPropertyChanged(self, propertySet, propertyName):
        propertyName = str(propertyName)

        if propertyName == 'neckYaw':
            self.tableMapping.neckYaw = self.params.getProperty('neckYaw')

        elif propertyName == 'lowerNeckPitch':
            self.tableMapping.lowerNeckPitch = self.params.getProperty('lowerNeckPitch')

    def addButtons(self):
        self.addManualSpacer()
        self.addManualButton('Read SDF & Sim', self.tableMapping.loadSDFFileAndRunSim)
        self.addManualSpacer()
        self.addManualButton('Set Head Position', self.tableMapping.setHeadPosition)

    def addTasks(self):
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
            addFunc('execute manip plan', self.tableMapping.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        self.taskTree.removeAllTasks()
        ###############

	#prep
	addFolder('Preparation')
        addFunc('fit table 1', functools.partial(self.tableMapping.userFitTable, tableNumber=1))
        addFunc('fit table 2', functools.partial(self.tableMapping.userFitTable, tableNumber=2))
        addFunc('store initial robot pose', self.tableMapping.initRobotPose)
        addManipTask('move hands down', self.tableMapping.planArmsDown, userPrompt=True)

        # walk to table
        addFolder('Walk to Table 1')
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame 1'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='table stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

	#survey table
	addFolder("Survey Table 1")
        addTask(SetNeckOrientation(name='look bottom right', angles=[60,-45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top right', angles=[45,-45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom half right', angles=[60,-25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top half right', angles=[45,-25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom center', angles=[60,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top center', angles=[45,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom half left', angles=[60,25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top half left', angles=[45,25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom left', angles=[60,45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top left', angles=[45,45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='reset neck position', angles=[0,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addFolder('Back Up')
        addFunc('create back up frame', self.tableMapping.computeBackUpStanceFrame)
        addTask(rt.RequestFootstepPlan(name='walk away from table', stanceFrameName='back up frame'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='walk away from table'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

        # walk to table
        addFolder('Walk to Table 2')
        addTask(rt.RequestFootstepPlan(name='plan walk to table', stanceFrameName='table stance frame 2'))
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        addTask(rt.CommitFootstepPlan(name='walk to table', planName='table stance frame footstep plan'))
        addTask(rt.WaitForWalkExecution(name='wait for walking'))

	#survey table
	addFolder("Survey Table 2")
        addTask(SetNeckOrientation(name='look bottom right', angles=[60,-45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top right', angles=[45,-45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom half right', angles=[60,-25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top half right', angles=[45,-25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom center', angles=[60,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top center', angles=[45,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom half left', angles=[60,25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top half left', angles=[45,25]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='look bottom left', angles=[60,45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
        addTask(SetNeckOrientation(name='look top left', angles=[45,45]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))

        addTask(SetNeckOrientation(name='reset neck position', angles=[0,0]))
	addTask(rt.DelayTask(name='sleep', delayTime=3.0))
