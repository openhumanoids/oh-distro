__author__ = 'manuelli'


import valkyriedriver
import valkyriedriverpanel
import forcevisualizer
import simplewalking
import LCMTuner

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel
from director import lcmUtils
import drc as lcmdrc
import os


def startupValkyrie(robotSystem, globalsDict=None):
    rs = robotSystem

    valkyrieDriver = valkyriedriver.ValkyrieDriver(robotSystem)
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)


    configFilename = 'valkyrie_director_vis_config.yaml'
    fullConfigFilename = os.getenv('DRC_BASE') + '/software/modules/mitdirector/config/' + configFilename
    forceVisualizer = forcevisualizer.ForceVisualizer(robotSystem, applogic.getDRCView(), fullConfigFilename)

    # # remove AtlasDriverPanel
    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    simpleWalking = simplewalking.SimpleWalking(robotSystem, valkyrieDriver)

    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel
        globalsDict['forceVisualizer'] = forceVisualizer
        globalsDict['simpleWalking'] = simpleWalking

        # add new task panel to global dict


def startupAtlas(robotSystem, globalsDict=None):

    # add force visualizer tool
    configFilename = 'atlas_director_vis_config.yaml'
    fullConfigFilename = os.getenv('DRC_BASE') + '/software/modules/mitdirector/config/' + configFilename
    forceVisualizer = forcevisualizer.ForceVisualizer(robotSystem, applogic.getDRCView(), fullConfigFilename)
    startupPlanner = StartupPlanner(robotSystem)


    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['forceVisualizer'] = forceVisualizer
        globalsDict['startupPlanner'] = startupPlanner

        # add new task panel to global dict


class StartupPlanner:
    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        lcmUtils.addSubscriber("START_MIT_STAND", lcmdrc.utime_t, self.executeManipPlanFromCurrentPose)

    def executeManipPlanFromCurrentPose(self, msg):
        q = self.robotSystem.robotStateJointController.q
        plan = self.robotSystem.ikPlanner.computePostureGoal(q,q)
        self.robotSystem.manipPlanner.commitManipPlan(plan)

        return plan
