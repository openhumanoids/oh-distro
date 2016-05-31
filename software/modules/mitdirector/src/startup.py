__author__ = 'manuelli'


import valkyriedriver
import valkyriedriverpanel

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel


def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    valkyrieDriver = valkyriedriver.ValkyrieDriver()
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)

    # # remove AtlasDriverPanel
    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel

        # add new task panel to global dict
