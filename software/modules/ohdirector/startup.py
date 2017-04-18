import valkyriedriver
import valkyriedriverpanel
import exampletaskpanel

from director import tasklaunchpanel
from director import applogic

def startup(robotSystem, globalsDict=None):

    valkyrieDriver = valkyriedriver.ValkyrieDriver()
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)

    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    # add a new task panel
    exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel
