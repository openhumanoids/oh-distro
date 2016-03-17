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
    FOOTSIZE_REDUCTION = 0.04
    FOOT_LENGTH = 0.25 - FOOTSIZE_REDUCTION
    FOOT_WIDTH = 0.15 - FOOTSIZE_REDUCTION
    FULL_FOOT_CONTACT_POINTS = np.array([[-0.5*FOOT_LENGTH, -0.5*FOOT_LENGTH, 0.5*FOOT_LENGTH, 0.5*FOOT_LENGTH],
                                  [0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH, 0.5*FOOT_WIDTH, -0.5*FOOT_WIDTH]])

    def __init__(self, robotStateModel, footstepsDriver, robotStateJointController, ikPlanner):
        self.footstepsDriver = footstepsDriver
        self.robotStateModel = robotStateModel
        self.robotStateJointController = robotStateJointController
        self.ikPlanner = ikPlanner

        # live operation flags
        self.chosenTerrain = 'simple'
        self.leadingFootByUser = 'Left'

        # Manual footsteps placement options
        self.nb_steps_ = 3
        self.forward_step_ = 0.30
        self.forward_balance_ = 0.05
        self.step_width_ = 0.27
        self.lateral_shift_ = 0.0


    def loadSDFFile(self):
        from director import sceneloader

        if self.chosenTerrain == 'simple':
            filename= os.environ['DRC_BASE'] + '/software/models/worlds/terrain_simple.sdf'
            sc=sceneloader.SceneLoader()
            sc.loadSDF(filename)

    def testManualWalking(self, leadFoot=None):
        if (leadFoot is None):
            if self.leadingFootByUser == 'Right':
                leadFoot=self.ikPlanner.rightFootLink #'r_foot'
            else:
                leadFoot=self.ikPlanner.leftFootLink #'l_foot'

        startPose = self.robotStateJointController.getPose('EST_ROBOT_STATE')

        self.manualFootstepsPlacement(leadFoot, removeFirstLeftStep = False, nextDoubleSupportPose = startPose)


    def manualFootstepsPlacement(self, standingFootName, removeFirstLeftStep = True, nextDoubleSupportPose = None):
        polyData = segmentation.getCurrentRevolutionData()

        obj = om.getOrCreateContainer('continuous')
        om.getOrCreateContainer('cont debug', obj)

        vis.updatePolyData( polyData, 'walking snapshot', parent='cont debug', visible=False)

        standingFootFrame = self.robotStateModel.getLinkFrame(standingFootName)
        vis.updateFrame(standingFootFrame, standingFootName, parent='cont debug', visible=False)

        # Step 1: Footsteps placement
        footsteps = self.placeStepsManually(removeFirstLeftStep, nextDoubleSupportPose)

        if not len(footsteps):
            print 'ERROR: No footsteps placed.'
            return

        # Step 2: Publish footsteps
        self.sendPlanningRequest(footsteps, nextDoubleSupportPose)


    def placeStepsManually(self, removeFirstLeftStep = True, nextDoubleSupportPose = None):
        
        # Place footsteps an the ground manually (no planning)
        # The number of footsteps, width and forward distance between steps are given by user.
        footsteps = []

        current_x_left = nextDoubleSupportPose[0]
        current_x_right = nextDoubleSupportPose[0]
        current_y_left = nextDoubleSupportPose[1]+((self.step_width_)/2)
        current_y_right = nextDoubleSupportPose[1]-((self.step_width_)/2)

        for i in range(self.nb_steps_):
            current_y_right = current_y_right + self.lateral_shift_
            current_y_left = current_y_left + self.lateral_shift_

            if self.leadingFootByUser == 'Right':
                current_x_right = current_x_right + self.forward_step_
                current_x_left = current_x_right + self.forward_balance_

                nextLeftTransform = transformUtils.frameFromPositionAndRPY([current_x_left,current_y_left,0.08], [0,0,0])
                nextRightTransform = transformUtils.frameFromPositionAndRPY([current_x_right,current_y_right,0.08], [0,0,0])

                footsteps.append(Footstep(nextRightTransform,True))
                footsteps.append(Footstep(nextLeftTransform,False))
            else:
                current_x_left = current_x_left + self.forward_step_
                current_x_right = current_x_left + self.forward_balance_

                nextLeftTransform = transformUtils.frameFromPositionAndRPY([current_x_left,current_y_left,0.08], [0,0,0])
                nextRightTransform = transformUtils.frameFromPositionAndRPY([current_x_right,current_y_right,0.08], [0,0,0])

                footsteps.append(Footstep(nextLeftTransform,False))
                footsteps.append(Footstep(nextRightTransform,True))

        if (removeFirstLeftStep is True):
            if (standingFootName is self.ikPlanner.rightFootLink ):
                footsteps = footsteps[1:]
                print "Removing the first left step"

        return footsteps


    def sendPlanningRequest(self, footsteps, nextDoubleSupportPose):

        goalSteps = []

        for i, footstep in enumerate(footsteps):
            step_t = footstep.transform

            step = lcmdrc.footstep_t()
            step.pos = positionMessageFromFrame(step_t)
            step.is_right_foot =  footstep.is_right_foot # flist[i,6] # is_right_foot
            step.params = self.footstepsDriver.getDefaultStepParams()

            # Visualization via triads
            #vis.updateFrame(step_t, str(i), parent="navigation")
            goalSteps.append(step)

        #nextDoubleSupportPose = self.robotStateJointController.q
        request = self.footstepsDriver.constructFootstepPlanRequest(nextDoubleSupportPose)
        request.num_goal_steps = len(goalSteps)
        request.goal_steps = goalSteps

        # force correct planning parameters:
        request.params.leading_foot = goalSteps[0].is_right_foot
        request.params.planning_mode = 1
        request.params.nom_forward_step = 0.38
        request.params.map_mode = 1 #  2 footplane, 0 h+n, 1 h+zup, 3 hori
        request.params.max_num_steps = len(goalSteps)
        request.params.min_num_steps = len(goalSteps)
        #request.default_step_params.support_contact_groups = self.FULL_FOOT_CONTACT_POINTS

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
        self.params.addProperty('Terrain Type', 0, attributes=om.PropertyAttributes(enumNames=['Ground','Simple']))
        self.params.addProperty('Leading Foot', 0, attributes=om.PropertyAttributes(enumNames=['Left','Right']))
        self.params.addProperty('Steps Number', 3, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=30, singleStep=1))
        self.params.addProperty('Forward Step', 0.30, attributes=om.PropertyAttributes(decimals=2, minimum=-0.5, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Forward Balance Step', 0.05, attributes=om.PropertyAttributes(decimals=2, minimum=-0.5, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Step Width', 0.27, attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=0.5, singleStep=0.01))
        self.params.addProperty('Lateral Shift', 0.0, attributes=om.PropertyAttributes(decimals=2, minimum=-0.5, maximum=0.5, singleStep=0.01))
        self._syncProperties()

    def onPropertyChanged(self, propertySet, propertyName):
        self._syncProperties()
        self.manualWalkingDemo.testManualWalking()

    def _syncProperties(self):
        if self.params.getPropertyEnumValue('Terrain Type') == 'Simple':
            self.manualWalkingDemo.chosenTerrain = 'simple'
        else:
            self.manualWalkingDemo.chosenTerrain = 'ground'        

        if self.params.getPropertyEnumValue('Leading Foot') == 'Left':
            self.manualWalkingDemo.leadingFootByUser = 'Left'
        else:
            self.manualWalkingDemo.leadingFootByUser = 'Right'
    
        self.manualWalkingDemo.nb_steps_ = self.params.getProperty('Steps Number')
        self.manualWalkingDemo.forward_step_ = self.params.getProperty('Forward Step')
        self.manualWalkingDemo.forward_balance_ = self.params.getProperty('Forward Balance Step')
        self.manualWalkingDemo.step_width_ = self.params.getProperty('Step Width')
        self.manualWalkingDemo.lateral_shift_ = self.params.getProperty('Lateral Shift')

    def addTasks(self):

        # some helpers
        def addTask(task, parent=None):
            self.taskTree.onAddTask(task, copy=False, parent=parent)

        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)

        mw = self.manualWalkingDemo

        ###############
        # add the tasks

        # load
        load = self.taskTree.addGroup('Loading')
        addFunc(functools.partial(mw.loadSDFFile), 'load scenario', parent=load)

        # plan walking
        load = self.taskTree.addGroup('Placing Footsteps')
        addFunc(functools.partial(mw.testManualWalking), 'place footsteps', parent=load)

