import bot_core as lcmbotcore
import drc as lcmdrc
from director import lcmUtils
from xml.dom import getDOMImplementation
import director.tasks.robottasks as rt
from director.tasks.taskuserpanel import TaskUserPanel
import functools
import datetime
import os
    

class TrajectoryTrackingTest(object):
    def __init__(self, ikPlanner, manipPlanner, robotStateJointController):
        self.ikPlanner = ikPlanner
        self.manipPlanner = manipPlanner
        self.robotStateJointController = robotStateJointController
        self.plans = []
    
    def planNominalPose(self):
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'General', 'handsdown both')
        self.plans.append(self.ikPlanner.computePostureGoal(startPose, endPose))

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])
    
    def planPose(self, poseName, side):
        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Trajectory Tracking Test', poseName, side)
        self.plans.append(self.ikPlanner.computePostureGoal(startPose, endPose))

class TrackingTestPanel(TaskUserPanel):

    def __init__(self, trajectoryTrackingTest):

        TaskUserPanel.__init__(self, windowTitle='Tacking Test')

        self.trackingTest = trajectoryTrackingTest
        self.addTasks()
    
    def addTasks(self):
        
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None, confirm=False):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
            if confirm:
                addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=parent)

        def addTest(name, pose, side):
            group = self.taskTree.addGroup(name, parent=None)
            groupNominal = self.taskTree.addGroup('plan nominal pose', parent=group)
            addFunc(self.trackingTest.planNominalPose, name='plan nominal pose', parent=groupNominal)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=groupNominal)
            addFunc(self.trackingTest.commitManipPlan, name='execute manip plan', parent=groupNominal)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=groupNominal)
            
            groupRequested = self.taskTree.addGroup('plan requested pose', parent=group)
            addFunc(functools.partial(self.trackingTest.planPose, pose, side), name='plan requested pose', parent=groupRequested)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=groupRequested)
            addFunc(self.trackingTest.commitManipPlan, name='execute manip plan', parent=groupRequested)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=groupRequested)
        
        addTest('raise left arm side', 'arm side', 'left')
        addTest('raise right arm side', 'arm side',  'right')
        addTest('raise left arm front', 'arm front', 'left')
        addTest('raise right arm front', 'arm front', 'right')
        
        groupNominal = self.taskTree.addGroup('plan nominal pose', parent=None)
        addFunc(self.trackingTest.planNominalPose, name='plan nominal pose', parent=groupNominal)
        addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=groupNominal)
        addFunc(self.trackingTest.commitManipPlan, name='execute manip plan', parent=groupNominal)
        addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=groupNominal)
        