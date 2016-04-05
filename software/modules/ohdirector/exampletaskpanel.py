from director import lcmUtils
from director import transformUtils
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
from director import ikplanner
from director.ikparameters import IkParameters
from director import vtkNumpy as vnp
from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit
import director.tasks.robottasks as rt

import os
import sys
import vtkAll as vtk
import numpy as np


class ExampleTaskPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner

    def test(self):
        print 'test'
        assert True


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, planner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.planner = planner

    def fit(self, polyData, points):
        pass


class ExampleTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Example Task')

        self.planner = ExampleTaskPlanner(robotSystem)
        self.fitter = ImageFitter(self.planner)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()

    def addButtons(self):
        self.addManualButton('example button', self.planner.test)

    def testPrint(self):
        self.appendMessage('test')

    def addDefaultProperties(self):
        self.params.addProperty('Example bool', True)
        self.params.addProperty('Example enum', 0, attributes=om.PropertyAttributes(enumNames=['Left', 'Right']))
        self.params.addProperty('Example double', 1.0, attributes=om.PropertyAttributes(singleStep=0.01, decimals=3))

    def onPropertyChanged(self, propertySet, propertyName):
          self.appendMessage('property changed: <b>%s</b>' % propertyName)
          self.appendMessage('  new value: %r' % self.params.getProperty(propertyName))

    def addTasks(self):

        ############
        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        self.taskTree.removeAllTasks()
        ###############


        # add the tasks

        addFolder('Tasks')
        addTask(rt.PrintTask(name='display message', message='hello world'))
        addTask(rt.DelayTask(name='wait', delayTime=1.0))
        addTask(rt.UserPromptTask(name='prompt for user input', message='please press continue...'))
        addFunc(self.planner.test, name='test planner')



