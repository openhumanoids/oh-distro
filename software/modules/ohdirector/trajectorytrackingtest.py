import bot_core as lcmbotcore
import drc as lcmdrc
from director import lcmUtils
from xml.dom import getDOMImplementation
import director.tasks.robottasks as rt
from director.tasks.taskuserpanel import TaskUserPanel
import functools
import datetime
import os

def addValue(element, value, data_format):
    format_string = '{{:s}} {{:{:s}}}'.format(data_format)
    element.firstChild.nodeValue = format_string.format(element.firstChild.nodeValue, value)
    

class TrajectoryTrackingTest(object):
    def __init__(self, ikPlanner, manipPlanner, robotStateJointController):
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.robotStateJointController = robotStateJointController
        self.executing = False
        self.record = False
        self.plans = []
        self.plan_joints = []
        self.execution_joints = []
        self.plan_time = dict()
        self.plan_positions = dict()
        self.execution_time = dict()
        self.execution_positions = dict()

        lcmUtils.addSubscriber('EST_ROBOT_STATE', lcmbotcore.robot_state_t, self.onEstimatedRobotState)
        lcmUtils.addSubscriber('PLAN_EXECUTION_STATUS', lcmdrc.plan_status_t, self.onPlanExecutionStatus)
        lcmUtils.addSubscriber('COMMITTED_ROBOT_PLAN', lcmdrc.robot_plan_t, self.onCommittedRobotPlan)
        
    def executeTest(self, poseName):
        if poseName:
            self.planNominalPose()            
    
    def planNominalPose(self):
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown both')
        self.record = False
        self.plans.append(self.ikPlanner.computePostureGoal(startPose, endPose))

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])
    
    def planPose(self, poseName, side):
        self.currentPose = poseName + '(' + side + ')'
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Trajectory Tracking Test', poseName, side)
        self.plans.append(self.ikPlanner.computePostureGoal(startPose, endPose))
        self.record = True
    
    def onCommittedRobotPlan(self, msg):
        if self.record:
            if not self.plan_joints:
                self.plan_joints = msg.plan[0].joint_name
            self.plan_time[self.currentPose] = []
            self.plan_positions[self.currentPose] = []
            self.execution_positions[self.currentPose] = []
            self.execution_time[self.currentPose] = []
            for p in msg.plan:
                self.plan_positions[self.currentPose].append(p.joint_position)
                self.plan_time[self.currentPose].append(p.utime)
    
    def onEstimatedRobotState(self, msg):
        if self.executing and self.record:
            if not self.execution_joints:
                self.hokuyo_joint = msg.joint_name.index('hokuyo_joint')
                joint_names = msg.joint_name
                joint_names.pop(self.hokuyo_joint)
                self.execution_joints = joint_names
            joint_positions = [msg.joint_position[position] for position in range(len(msg.joint_position)) if position != self.hokuyo_joint];
            self.execution_positions[self.currentPose].append(joint_positions)
            self.execution_time[self.currentPose].append(msg.utime)
    
    def onPlanExecutionStatus(self, msg):
        if msg.execution_status == msg.EXECUTION_STATUS_EXECUTING:
            self.executing = True
        else:
            self.executing = False
    
    def closeFile(self):
        impl = getDOMImplementation()
        document = impl.createDocument(None, 'results', None)
        root = document.documentElement
        createdElement = document.createElement('created')
        createdElement.appendChild(document.createTextNode(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
        root.appendChild(createdElement)
        
        for poseName in self.plan_positions:
            testElement = document.createElement('test')
            testElement.setAttribute('name', poseName)
            root.appendChild(testElement)
            
            committedPlanElement = document.createElement('committed_plan')
            testElement.appendChild(committedPlanElement)
            
            executedPlanElement = document.createElement('executed_plan')
            testElement.appendChild(executedPlanElement)
            
            for joint in range(len(self.plan_joints)):
                committedPlanPositionElement = document.createElement('position')
                committedPlanPositionElement.setAttribute('joint_name', self.plan_joints[joint])
                committedPlanPositionElement.appendChild(document.createTextNode(''))
                committedPlanElement.appendChild(committedPlanPositionElement)
                
                for p in range(len(self.plan_positions[poseName])):
                    addValue(committedPlanPositionElement, self.plan_positions[poseName][p][joint], '.15g')
                
                executedPlanPositionElement = document.createElement('position')
                executedPlanPositionElement.setAttribute('joint_name', self.execution_joints[joint])
                executedPlanPositionElement.appendChild(document.createTextNode(''))
                executedPlanElement.appendChild(executedPlanPositionElement)
                
                for p in range(len(self.execution_positions[poseName])):
                    addValue(executedPlanPositionElement, self.execution_positions[poseName][p][joint], '.15g')
        
            committedPlanTimeElement = document.createElement('time')
            committedPlanTimeElement.appendChild(document.createTextNode(''))
            committedPlanElement.appendChild(committedPlanTimeElement)
            for t in self.plan_time[poseName]:
                addValue(committedPlanTimeElement, t, 'd')
        
            executedPlanTimeElement = document.createElement('time')
            executedPlanTimeElement.appendChild(document.createTextNode(''))
            executedPlanElement.appendChild(executedPlanTimeElement)
            for t in self.execution_time[poseName]:
                addValue(executedPlanTimeElement, t, 'd')

        outputFile = open(os.path.expanduser('~') + '/trajectory_tracking_output', 'w')
        outputFile.write(document.toprettyxml())
        outputFile.close()

class TrackingTestPanel(TaskUserPanel):

    def __init__(self, trajectoryTrackingTest):

        TaskUserPanel.__init__(self, windowTitle='Tacking Test')

        self.trackingTest = trajectoryTrackingTest

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()
    
    def addDefaultProperties(self):
        pass
    
    def addButtons(self):
        pass
    
    def addTasks(self):
        
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None, confirm=False):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=parent)

        def addTest(name, side):
            group = self.taskTree.addGroup(name + '(' + side + ')', parent=None)
            addFunc(self.trackingTest.planNominalPose, name='plan nominal pose', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(self.trackingTest.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)
            
            addFunc(functools.partial(self.trackingTest.planPose, name, side), name='plan requested pose', parent=group)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=group)
            addFunc(self.trackingTest.commitManipPlan, name='execute manip plan', parent=group)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=group)
        
        addTest('arm up', 'left')
#         addTest('arm up', 'right')
        addFunc(self.trackingTest.closeFile, name = 'save results', parent = 'arm up (right)')
        