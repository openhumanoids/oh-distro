__author__ = 'manuelli'

import time

from director import transformUtils
from director import frameupdater
from director import visualization as vis
import vtkAll as vtk
from director import timercallback



class SimpleWalking:

    def __init__(self, robotSystem_, valkyrieDriver):
        self.robotSystem = robotSystem_
        self.valkyrieDriver = valkyrieDriver

        self.startWalkingGoalFrame = None
        self.endWalkingGoalFrame = None
        self.initializeOptions()
        self.currentState = 'forwards' # can only be forwards or backwards

        self.timer = timercallback.TimerCallback(targetFps=0.5, callback=self.callback)

    def switchState(self):
        if self.currentState == 'forwards':
            self.currentState = 'backwards'
        else:
            self.currentState = 'forwards'

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def initializeOptions(self):
        self.options = dict()
        self.options['walkingDistance'] = 1.5

    # get and save a frame between feet
    def initialize(self):
        self.startWalkingGoalFrame = self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel)
        self.endWalkingGoalFrame = transformUtils.copyFrame(self.startWalkingGoalFrame)

        self.endWalkingGoalFrame.Translate(self.options['walkingDistance'],0,0)


        vis.updateFrame(self.startWalkingGoalFrame, 'start walking goal frame', scale=0.2)
        vis.updateFrame(self.endWalkingGoalFrame, 'end walking goal frame', scale=0.2)


    def planWalking(self, direction='forwards'):
        goalFrame = None

        if direction=='forwards':
            goalFrame = self.endWalkingGoalFrame
            stanceFrameName = 'end walking goal frame'
        elif direction=='backwards':
            goalFrame = self.startWalkingGoalFrame
            stanceFrameName = 'start walking goal frame'
        else:
            raise ValueError('direction must be either forwards or backwards')


        # plan footsteps
        pose = self.robotSystem.robotStateJointController.q.copy()
        request = self.robotSystem.footstepsDriver.constructFootstepPlanRequest(pose, goalFrame)
        footstepPlan = self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)


        # commit the footstep plan
        self.robotSystem.footstepsDriver.commitFootstepPlan(footstepPlan)



    def checkIfCurrentPlanFinished(self):
        planFinished = False

        planStatusMsg = self.valkyrieDriver.lastPlanStatusMsg

        if planStatusMsg is None:
            return True

        if ((planStatusMsg.plan_type == planStatusMsg.UNKNOWN) or
                (planStatusMsg.execution_status == planStatusMsg.EXECUTION_STATUS_FINISHED)):
            planFinished = True

        return planFinished

    def callback(self):
        # check to see if current plan is finished, or there is no plan
        planFinished = self.checkIfCurrentPlanFinished()

        if (planFinished):
            print "making a new walking plan"
            self.switchState()
            self.planWalking()
