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
        self.ui.calibrateEncodersButton.connect('clicked()', self.onCalibrateEncoders)
        self.ui.prepButton.connect('clicked()', self.onPrep)
        self.ui.initNavButton.connect('clicked()', self.onInitNav)
        self.ui.combinedStandButton.connect('clicked()', self.onCombinedStand)
        self.ui.stopButton.connect('clicked()', self.onStop)
        self.ui.freezeButton.connect('clicked()', self.onValkyrieFreeze)


        self.ui.tareFTButton.connect('clicked()', self.onTareFT)
        self.ui.forceControlButton.connect('clicked()', self.onForceControl)
        self.ui.positionControlButton.connect('clicked()', self.onPositionControl)


        self.updateTimer = TimerCallback(targetFps=5)
        self.updateTimer.callback = self.updatePanelValkyrie
        self.updatePanelValkyrie()
        self.updateTimer.start()


    def updatePanelAtlas(self):
        self.updateBehaviorLabel()
        self.updateControllerStatusLabel()
        self.updateRecoveryEnabledLabel()
        self.updateBracingEnabledLabel()
        self.updateBatteryStatusLabel()
        self.updateStatus()
        self.updateButtons()
        self.updateElectricArmStatus()
        self.driver.updateCombinedStandLogic()

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

    def updateStatus(self):
        self.ui.inletPressure.value = self.driver.getCurrentInletPressure()
        self.ui.supplyPressure.value = self.driver.getCurrentSupplyPressure()
        self.ui.returnPressure.value = self.driver.getCurrentReturnPressure()
        self.ui.airSumpPressure.value = self.driver.getCurrentAirSumpPressure()
        self.ui.pumpRpm.value =  self.driver.getCurrentPumpRpm()
        self.ui.maxActuatorPressure.value = self.driver.getMaxActuatorPressure()

    def getElectricArmCheckBoxes(self):
        return [self.ui.armCheck1,
                  self.ui.armCheck2,
                  self.ui.armCheck3,
                  self.ui.armCheck4,
                  self.ui.armCheck5,
                  self.ui.armCheck6]

    def setupElectricArmCheckBoxes(self):
        for check in self.getElectricArmCheckBoxes():
            check.connect('clicked()', self.onEnableElectricArmChecked)

    def updateElectricArmStatus(self):

        temps = [self.ui.armTemp1,
                  self.ui.armTemp2,
                  self.ui.armTemp3,
                  self.ui.armTemp4,
                  self.ui.armTemp5,
                  self.ui.armTemp6]

        for i, check in enumerate(self.getElectricArmCheckBoxes()):
            enabled = self.driver.getElectricArmEnabledStatus(i)
            check.setText('yes' if enabled else 'no')

        for i, temp in enumerate(temps):
            temp.setValue(self.driver.getElectricArmTemperature(i))

    def updateButtons(self):

        behavior = self.driver.getCurrentBehaviorName()
        behaviorIsFreeze = behavior == 'freeze'

        self.ui.calibrateNullBiasButton.setEnabled(behaviorIsFreeze)
        self.ui.calibrateElectricArmsButton.setEnabled(behaviorIsFreeze)
        self.ui.calibrateEncodersButton.setEnabled(behaviorIsFreeze)
        self.ui.prepButton.setEnabled(behaviorIsFreeze)
        self.ui.standButton.setEnabled(behavior in ('prep', 'stand', 'user', 'manip', 'step', 'walk'))
        self.ui.mitStandButton.setEnabled(behavior=='user')
        self.ui.manipButton.setEnabled(behavior in ('stand', 'manip'))
        self.ui.userButton.setEnabled(behavior is not None)

    def onEnableElectricArmChecked(self):
        enabledState = [bool(check.checked) for check in self.getElectricArmCheckBoxes()]
        self.driver.sendElectricArmEnabledState(enabledState)

    def onFreeze(self):
        self.driver.sendFreezeCommand()

    def onStop(self):
        self.driver.sendStopCommand()

    def onCalibrateEncoders(self):
        self.driver.sendCalibrateEncodersCommand()

    def onCalibrateNullBias(self):
        self.driver.sendCalibrateNullBiasCommand()

    def onCalibrateElectricArms(self):
        self.driver.sendCalibrateElectricArmsCommand()

    def onInitNav(self):
        self.driver.sendInitAtZero()

    def onTareFT(self):
        self.driver.sendTareFT()

    def onPrep(self):
        self.driver.sendPrepCommand()

    def onStand(self):
        self.driver.sendStandCommand()

    def onCombinedStand(self):
        self.driver.sendCombinedStandCommand()

    def onMITStand(self):
        self.driver.sendMITStandCommand()

    def onManip(self):
        self.driver.sendManipCommand()

    def onUser(self):
        self.driver.sendUserCommand()

    def sendCustomPressure(self):
        self.driver.sendDesiredPumpPsi(self.ui.customPumpPressure.value)


    # valkyrie specific
    def onForceControl(self):
        self.driver.sendForceControlCommand(self.ui.transitionTimeSpinBox.value)

    def onPositionControl(self):
        self.driver.sendPositionControlCommand(self.ui.transitionTimeSpinBox.value)

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
