import os
import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director.timercallback import TimerCallback
from director import drcargs

from functools import partial


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class ValkyrieDriverPanel(object):

    def __init__(self, driver):

        self.driver = driver

        loader = QtUiTools.QUiLoader()

        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), '../ui/ddValkyrieDriverPanel.ui'))

        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        # Main Panel

        self.widget.setWindowTitle('Valkyrie Driver Panel')
        self.ui.initNavButton.connect('clicked()', self.onInitNav)
        self.ui.initNavButton.connect('clicked()', self.onInitNav)
        self.ui.standButton.connect('clicked()', self.onStand)
        self.ui.stopButton.connect('clicked()', self.onStop)
        self.ui.servoButton.connect('clicked()', self.onServo)
        self.ui.freezeButton.connect('clicked()', self.onValkyrieFreeze)
        self.ui.standPrepButton.connect('clicked()', self.onStandPrep)

        self.ui.tareFTButton.connect('clicked()', self.onTareFT)
        self.ui.forceControlButton.connect('clicked()', self.onForceControl)
        self.ui.positionControlButton.connect('clicked()', self.onPositionControl)
        self.ui.standPrepModeButton.connect('clicked()', self.onStandPrepMode)


        self.updateTimer = TimerCallback(targetFps=5)
        self.updateTimer.callback = self.updatePanelValkyrie
        self.updatePanelValkyrie()
        self.updateTimer.start()


    def updatePanelValkyrie(self):
        self.updateValkyrieLCM2ROSControlLabel()
        self.updateRecoveryEnabledLabel()
        self.updateBracingEnabledLabel()

    def updateValkyrieLCM2ROSControlLabel(self):
        self.ui.LCM2ROSControlBehaviorLabel.text = self.driver.getCurrentBehaviorName() or '<unknown>'

    def updateBehaviorLabel(self):
        self.ui.behaviorLabel.text = self.driver.getCurrentBehaviorName() or '<unknown>'

    def updateControllerStatusLabel(self):
        self.ui.controllerStatusLabel.text = self.driver.getControllerStatus() or '<unknown>'

    def updateBatteryStatusLabel(self):
        charge = self.driver.getBatteryChargeRemaining()
        self.ui.batteryStatusLabel.text = '<unknown>' if charge is None else '%d%%' % charge

    def updateRecoveryEnabledLabel(self):
        self.ui.recoveryEnabledLabel.text = self.driver.getRecoveryEnabledStatus() or '<unknown>'

    def updateBracingEnabledLabel(self):
        self.ui.bracingEnabledLabel.text = self.driver.getBracingEnabledStatus() or '<unknown>'

    def onFreeze(self):
        self.driver.sendFreezeCommand()

    def onStop(self):
        self.driver.sendStopCommand()

    def onInitNav(self):
        self.driver.sendInitAtZero()

    def onTareFT(self):
        self.driver.sendTareFT()

    def onServo(self):
        self.driver.sendServoPlan()

    def onStandPrep(self):
        self.driver.sendStandPrepPlan()

    def onStand(self):
        self.driver.sendStandPlan()

    # valkyrie specific
    def onForceControl(self):
        self.driver.sendForceControlCommand(self.ui.transitionTimeSpinBox.value)

    def onPositionControl(self):
        self.driver.sendPositionControlCommand(self.ui.transitionTimeSpinBox.value)

    def onStandPrepMode(self):
        self.driver.sendStandPrepCommand(self.ui.transitionTimeSpinBox.value)

    def onValkyrieFreeze(self):
        self.driver.sendValkyrieFreezeCommand()




def _getAction():

    actionName = 'ActionValkyrieDriverPanel'
    action = app.getToolBarActions().get(actionName)

    if action is None:

        icon = QtGui.QIcon(os.path.join(os.path.dirname(__file__), '../images/nasa_logo.png'))
        assert os.path.isfile(os.path.join(os.path.dirname(__file__), '../images/nasa_logo.png'))

        action = QtGui.QAction(icon, 'Valkyrie Driver Panel', None)
        action.objectName = 'ActionValkyrieDriverPanel'
        action.checkable = True

        mainWindow = app.getMainWindow()
        toolbar = mainWindow.panelToolBar()

        toolbar.insertAction(toolbar.actions()[0], action)

    return action


def init(driver):

    global panel
    global dock

    panel = ValkyrieDriverPanel(driver)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()


    return panel
