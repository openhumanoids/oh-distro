import valkyriedriver
import valkyriedriverpanel
import exampletaskpanel
import tableboxdemo

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel

import manualwalkingdemo


def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    valkyrieDriver = valkyriedriver.ValkyrieDriver()
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)

    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)

    tableboxDemo = tableboxdemo.TableboxDemo(rs.robotStateModel, rs.playbackRobotModel,
                    rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                    rs.view, rs.robotStateJointController)
    tableboxTaskPanel = tableboxdemo.TableboxTaskPanel(tableboxDemo)
    tasklaunchpanel.panel.addTaskPanel('Tablebox', tableboxTaskPanel.widget)

    manualWalkingDemo = manualwalkingdemo.ManualWalkingDemo(rs.robotStateModel,
                    rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner)
    manualWalkingTaskPanel = manualwalkingdemo.ManualWalkingTaskPanel(manualWalkingDemo)
    tasklaunchpanel.panel.addTaskPanel('Manual Walking', manualWalkingTaskPanel.widget)

    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel
        globalsDict['tableboxDemo'] = tableboxDemo
        globalsDict['manualWalkingDemo'] = manualWalkingDemo



