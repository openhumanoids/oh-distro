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
from director.utime import getUtime
from director.lcmframe import positionMessageFromFrame

import drc as lcmdrc
import ihmc as lcmihmc


class DelayExecution(rt.AsyncTask):
    waitingTime = 3.0; # safe default

    def run(self):
        yield rt.DelayTask(delayTime=self.waitingTime).run()

class Footstep():
    def __init__(self, transform, is_right_foot):
        self.transform = transform
        self.is_right_foot = is_right_foot

class CalisthenicsDemo(object):
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
        self.footTrajectoryTime = 3.0 # safe default

    def commitManipPlan(self):
        self.manipPlanner.commitManipPlan(self.plans[-1])

    def makeFootFrame(self, side):
        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        footTransform = self.robotStateModel.getLinkFrame(footLink)
        vis.showFrame(footTransform, side + " foot goal")

    def makeLeftFootFrame(self):
        self.makeFootFrame("left")

    def makeRightFootFrame(self):
        self.makeFootFrame("right")

    def moveRightFootToFrame(self):
        self.moveFootToGoalFrame("right")

    def moveFootToGoalFrame(self, side):
        if side == 'left':
            footFrame = om.findObjectByName('left foot goal')
        else:
            footFrame = om.findObjectByName('right foot goal')

        self.moveFootToFrame(footFrame.transform, side)


    def getCurrentRelativeFootFrame(self, side = 'right'):
        # Get the transform between the standing foot (otherSide) and the raised foot (side)
        side = 'right'
        otherSide = 'left'

        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        footTransform = self.robotStateModel.getLinkFrame(footLink)
        vis.updateFrame(footTransform, "current foot frame")

        otherFootLink = self.ikPlanner.leftFootLink if otherSide == 'left' else self.ikPlanner.rightFootLink
        otherFootTransform = self.robotStateModel.getLinkFrame(otherFootLink)
        vis.updateFrame(otherFootTransform, "other foot frame")

        relativeFootTransform = transformUtils.copyFrame(footTransform)
        relativeFootTransform.PreMultiply()
        relativeFootTransform.Concatenate(otherFootTransform.GetLinearInverse())

        [ pos, quat ] = transformUtils.poseFromTransform(relativeFootTransform)

        print 'Relative Transform from', footLink, 'to', otherFootLink
        print pos
        print quat


    def moveFootToSkater(self):
        floatingFoot = 'right'
        pos = [-0.79294841, -0.1598023,  0.4183836 ]
        quat = [ 0.63925192, -0.00519966,  0.76897871, -0.00129613]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToPreSkater(self):
        floatingFoot = 'right'
        pos = [-0.45855504, -0.15881477,  0.19467936]
        quat = [  8.49938853e-01,  -3.58606857e-03,   5.26868563e-01,   7.76788522e-04]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToJustOffGround(self):
        floatingFoot = 'right'
        pos = [ 0.0, -0.24, 0.05]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToOnGround(self):
        floatingFoot = 'right'
        pos = [ 0.0, -0.24, 0.00]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToBelowGround(self):
        floatingFoot = 'right'
        pos = [ 0.0, -0.24, -0.02]
        quat = [ 1,0,0,0 ]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToSuperman(self):
        floatingFoot = 'right'
        pos = [ -0.00222505, -0.64204418, 0.19250567]
        quat = [ 9.71305230e-01, -2.37834617e-01, 9.18712295e-04, -2.36906030e-05]
        self.moveFootToRelativeFrame(transformUtils.transformFromPose(pos, quat), floatingFoot)

    def moveFootToRelativeFrame(self, relativeFootTransform, side = 'right'):
        # move side to this transfrom relative to the otherSide (which we are standing on)       
        side = 'right'
        otherSide = 'left'

        otherFootLink = 'right' if otherSide == 'left' else 'left'

        otherFootLink = self.ikPlanner.leftFootLink if otherSide == 'left' else self.ikPlanner.rightFootLink
        otherFootTransform = self.robotStateModel.getLinkFrame(otherFootLink)
        vis.updateFrame(otherFootTransform, "other foot frame")

        goalFootTransform = transformUtils.copyFrame(otherFootTransform)
        goalFootTransform.PreMultiply()
        goalFootTransform.Concatenate(relativeFootTransform)

        self.moveFootToFrame(goalFootTransform, side)

    def moveFootToFrame(self, goalFootTransform, side):
        footLink = self.ikPlanner.leftFootLink if side == 'left' else self.ikPlanner.rightFootLink
        currenFootTransform = self.robotStateModel.getLinkFrame(footLink)

        msg = lcmihmc.foot_pose_packet_message_t()
        msg.utime = getUtime()
        [msg.position, msg.orientation] = transformUtils.poseFromTransform(goalFootTransform)

        # Rule of thumb speed limit: 10cm/sec
        distanceToMoveFoot = np.linalg.norm(tuple(np.subtract(currenFootTransform.GetPosition(), goalFootTransform.GetPosition() )) )
        self.footTrajectoryTime = distanceToMoveFoot*10.0

        # For Controller
        msg.trajectory_time = self.footTrajectoryTime 
        # For autonomy, add this time to account for delays
        DelayExecution.waitingTime = self.footTrajectoryTime + 4.0 

        if side == 'left':
            msg.robot_side = 0
            lcmUtils.publish("DESIRED_LEFT_FOOT_POSE", msg)
        else:
            msg.robot_side = 1
            lcmUtils.publish("DESIRED_RIGHT_FOOT_POSE", msg)


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

    def planSkaterPrep(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Calisthenics', 'skater_prep')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

    def planNominal(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Calisthenics', 'nominal full posture')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)        

    def planSupermanPrep(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Calisthenics', 'superman_1')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)        

    def planSupermanArms(self):
        startPose = self.getPlanningStartPose()
        endPose = self.ikPlanner.getMergedPostureFromDatabase(startPose, 'Calisthenics', 'superman_arms_and_back')
        newPlan = self.ikPlanner.computePostureGoal(startPose, endPose)
        self.addPlan(newPlan)

class CalisthenicsTaskPanel(TaskUserPanel):

    def __init__(self, calisthenicsDemo):

        TaskUserPanel.__init__(self, windowTitle='Walking Task')

        self.calisthenicsDemo = calisthenicsDemo

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('Create Left Frame', self.calisthenicsDemo.makeLeftFootFrame)       
        self.addManualButton('Create Right Frame', self.calisthenicsDemo.makeRightFootFrame)
        self.addManualButton('Move Right Foot', self.calisthenicsDemo.moveRightFootToFrame)

        self.addManualSpacer()
        self.addManualButton('RFoot Below', self.calisthenicsDemo.moveFootToBelowGround)
        self.addManualButton('RFoot Just Above', self.calisthenicsDemo.moveFootToJustOffGround)
        self.addManualButton('RFoot Pre Skater', self.calisthenicsDemo.moveFootToPreSkater)
        self.addManualButton('RFoot Skater', self.calisthenicsDemo.moveFootToSkater)


    def addDefaultProperties(self):
        #self.params.addProperty('Leading Foot', 1, attributes=om.PropertyAttributes(enumNames=['Left','Right']))

        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()


    def _syncProperties(self):
        x = 1


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
            addFunc('execute manip plan', self.calisthenicsDemo.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder

        def addFootTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        c = self.calisthenicsDemo
        self.taskTree.removeAllTasks()

        ######################### Script 1
        addFolder('skater on left foot')
        addManipTask('move into skater prep', c.planSkaterPrep, userPrompt=True)

        addFunc('foot off ground', c.moveFootToJustOffGround)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot pre skater', c.moveFootToPreSkater)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot skater', c.moveFootToSkater)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot pre skater', c.moveFootToPreSkater)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot off ground', c.moveFootToJustOffGround)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot below ground', c.moveFootToBelowGround)
        addTask(DelayExecution(name='wait for foot'))
        addManipTask('return to nominal', c.planNominal, userPrompt=True)        

        ######################### Script 2
        addFolder('superman on left foot')
        addManipTask('move into superman prep', c.planSupermanPrep, userPrompt=True)
        addManipTask('superman arms and back', c.planSupermanArms, userPrompt=True)

        addFunc('foot off ground', c.moveFootToJustOffGround)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot to superman', c.moveFootToSuperman)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot off ground', c.moveFootToJustOffGround)
        addTask(DelayExecution(name='wait for foot'))

        addFunc('foot below ground', c.moveFootToBelowGround)
        addTask(DelayExecution(name='wait for foot'))
        addManipTask('return to nominal', c.planNominal, userPrompt=True)