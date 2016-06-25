import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director.utime import getUtime
from director import transformUtils
from director import lcmUtils

import os
import bot_core as lcmbotcore

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
        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), 'ui/ddValkyrieDriverPanel.ui'))
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Valkyrie Driver Panel')
        self.ui = WidgetDict(self.widget.children())

        # Main Panel
        self.ui.sendButton.connect('clicked()', self.onSendWholeBodyCommand)
        self.ui.modeBox.connect('currentIndexChanged(const QString&)', self.onModeBox)
        self.wholeBodyMode='Whole Body'

        self.ui.lFootUpButton.connect('clicked()', self.moveLeftFootUpButtonClicked)
        self.ui.lFootDownButton.connect('clicked()', self.moveLeftFootDownButtonClicked)
        self.ui.rFootUpButton.connect('clicked()', self.moveRightFootUpButtonClicked)
        self.ui.rFootDownButton.connect('clicked()', self.moveRightFootDownButtonClicked)

        # Neck Control
        self.ui.parkNeckButton.connect('clicked()', self.driver.sendParkNeckCommand)
        self.ui.setNeckPitchButton.connect('clicked()', self.setNeckPitchButtonClicked)

        # Hand Control
        self.ui.sendHandCommandButton.connect('clicked()', self.sendHandCommandButtonClicked)
        self.ui.openHandButton.connect('clicked()', self.openHandButtonClicked)


    def getModeInt(self, inputStr):
        if inputStr == 'Whole Body':
            return 0
        if inputStr == 'Left Arm':
            return 1
        if inputStr == 'Right Arm':
            return 2
        if inputStr == 'Both Arms':
            return 3
        return 0

    def onSendWholeBodyCommand(self):
        self.driver.sendWholeBodyCommand( self.getModeInt(self.wholeBodyMode) )

    def getComboText(self, combo):
        return str(combo.currentText)

    def onModeBox(self):
        self.wholeBodyMode = self.getComboText(self.ui.modeBox)


    def moveLeftFootUpButtonClicked(self):
        self.moveFoot([0,0,0.05], 'left')

    def moveLeftFootDownButtonClicked(self):
        self.moveFoot([0,0,-0.05], 'left')

    def moveRightFootUpButtonClicked(self):
        self.moveFoot([0,0,0.05], 'right')

    def moveRightFootDownButtonClicked(self):
        self.moveFoot([0,0,-0.05], 'right')

    def moveFoot(self, offset, side):
        msg = lcmihmc.foot_pose_packet_message_t()
        msg.utime = getUtime();
        foot_link = self.driver.ikPlanner.leftFootLink if side == 'left' else self.driver.ikPlanner.rightFootLink

        footTransform = self.driver.ikPlanner.robotModel.getLinkFrame(foot_link)
        footOffsetTransform = transformUtils.frameFromPositionAndRPY(offset, [0., 0., 0.])
        footTransform.PreMultiply()
        footTransform.Concatenate(footOffsetTransform)

        [msg.position, msg.orientation] = transformUtils.poseFromTransform(footTransform)

        msg.trajectory_time = 2.0
        if side == 'left':
            msg.robot_side = 0
            lcmUtils.publish("DESIRED_LEFT_FOOT_POSE", msg)
        else:
            msg.robot_side = 1
            lcmUtils.publish("DESIRED_RIGHT_FOOT_POSE", msg)

    def setNeckPitchButtonClicked(self):
        self.driver.setNeckPitch(self.ui.lowerNeckPitchSpinBox.value)

    def sendHandCommandButtonClicked(self):
        side = self.ui.handSelectorComboBox.currentText.lower()
        thumbRoll = float(self.ui.thumbRollSlider.value) / 99.
        thumbPitch1 = float(self.ui.thumbPitch1Slider.value) / 99.
        thumbPitch2 = float(self.ui.thumbPitch2Slider.value) / 99.
        indexFingerPitch = float(self.ui.indexFingerPitchSlider.value) / 99.
        middleFingerPitch = float(self.ui.middleFingerPitchSlider.value) / 99.
        pinkyPitch = float(self.ui.pinkyPitchSlider.value) / 99.
        self.driver.sendHandCommand(side, thumbRoll, thumbPitch1, thumbPitch2, indexFingerPitch, middleFingerPitch, pinkyPitch)

    def openHandButtonClicked(self):
        side = self.ui.handSelectorComboBox.currentText.lower()
        self.driver.openHand(side)


def _getAction():

    actionName = 'ActionValkyrieDriverPanel'
    action = app.getToolBarActions().get(actionName)

    if action is None:

        icon = QtGui.QIcon(os.path.join(os.path.dirname(__file__), 'images/nasa_logo.png'))
        assert os.path.isfile(os.path.join(os.path.dirname(__file__), 'images/nasa_logo.png'))

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
