#!/usr/bin/env python

from PyQt4 import QtCore, QtGui, uic

import lcm
import sys
import json
import time
import math
import os
import drc as lcmdrc

# allow control-c to kill the program
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)


class LCMWrapper(object):
    '''
    A basic class providing some convenience methods around LCM.
    '''

    def __init__(self):
        self.lc = lcm.LCM()

    def subscribe(self, channel, callback):
        return self.lc.subscribe(channel, callback)

    def captureMessage(self, channel, messageClass):

        messages = []
        def handleMessage(channel, messageData):
            messages.append(messageClass.decode(messageData))

        subscription = self.subscribe(channel, handleMessage)

        while not messages:
            self.lc.handle()
            time.sleep(0.01)

        self.lc.unsubscribe(subscription)
        return messages[0]

    def publish(self, channel, message):
        self.lc.publish(channel, message.encode())


def captureRobotState():
    '''
    Blocks until a new LCM message is received on the EST_ROBOT_STATE channel,
    returns the new message.
    '''
    return lcmWrapper.captureMessage('EST_ROBOT_STATE', lcmdrc.robot_state_t)


def getUtime():
    return int(time.time() * 1e6)


def publishDrivingCommand(commandType, data, channel='DRILL_CONTROL'):
    '''
    '''
    msg = lcmdrc.drill_control_t()
    msg.utime = getUtime()
    msg.control_type = commandType
    msg.data = data
    msg.data_length = len(data)
    lcmWrapper.publish(channel, msg)


class DrivingPanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.setup()

    def setup(self):
        self.ui.connect(self.ui.steeringSlider, QtCore.SIGNAL('valueChanged(int)'), self.onSteeringSliderValueChanged)
        self.ui.connect(self.ui.setSteeringDepthButton, QtCore.SIGNAL('clicked()'), self.onSetSteeringDepthClicked)
        self.ui.connect(self.ui.refitSteeringWheelButton, QtCore.SIGNAL('clicked()'), self.onRefitSteeringWheelClicked)

        self.ui.connect(self.ui.throttlePulseButton, QtCore.SIGNAL('clicked()'), self.onThrottlePulseClicked)
        self.ui.connect(self.ui.grabCurrentButton, QtCore.SIGNAL('clicked()'), self.onGrabCurrentClicked)
        self.ui.connect(self.ui.setLegZeroButton, QtCore.SIGNAL('clicked()'), self.onSetLegZeroClicked)

        self.ui.connect(self.ui.hipXSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)
        self.ui.connect(self.ui.hipYSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)
        self.ui.connect(self.ui.hipZSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)
        self.ui.connect(self.ui.kneeSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)
        self.ui.connect(self.ui.ankleXSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)
        self.ui.connect(self.ui.ankleYSlider, QtCore.SIGNAL('valueChanged(int)'), self.onLegTeleop)

        self.onSteeringSliderValueChanged()


    def getSteeringValue(self):
        return  -1 + self.ui.steeringSlider.value() / 50.0

    def getThrottleDuration(self):
        return self.ui.throttleDurationSpin.value()

    def getThrottleAnkleDegrees(self):
        return self.ui.throttleAnkleSpin.value()

    def getSteeringDepth(self):
        return self.ui.steeringDepthSpin.value() / 100.0

    def getAutoCommitIsEnabled(self):
        return self.ui.autoCommitCheckBox.isChecked()

    def onSteeringSliderValueChanged(self):
        self.ui.steeringLabel.setText(str(self.getSteeringValue()))
        publishDrivingCommand(lcmdrc.drill_control_t.SET_STEERING_ANGLE, [self.getSteeringValue()])

    def onRefitSteeringWheelClicked(self):
        publishDrivingCommand(lcmdrc.drill_control_t.REFIT_STEERING, [])

    def onSetSteeringDepthClicked(self):
        publishDrivingCommand(lcmdrc.drill_control_t.SET_STEERING_DEPTH, [self.getSteeringDepth()])

    def onThrottlePulseClicked(self):

        duration = self.getThrottleDuration()
        ankleRadians = math.radians(self.getThrottleAnkleDegrees())
        autoCommit = self.getAutoCommitIsEnabled()

        data = [ankleRadians, duration, float(autoCommit)]
        publishDrivingCommand(lcmdrc.drill_control_t.DRIVING_PULSE, data)

    def onGrabCurrentClicked(self):
        self.setLegJointPositionsFromRobotState()
        self.updateLegTeleopLabels()

    def getLegTeleopSliders(self):
        return [
            self.ui.hipXSlider,
            self.ui.hipYSlider,
            self.ui.hipZSlider,
            self.ui.kneeSlider,
            self.ui.ankleXSlider,
            self.ui.ankleYSlider
            ]

    def getLegTeleopLabels(self):
        return [
            self.ui.hipXLabel,
            self.ui.hipYLabel,
            self.ui.hipZLabel,
            self.ui.kneeLabel,
            self.ui.ankleXLabel,
            self.ui.ankleYLabel
            ]

    def updateLegTeleopLabels(self):

        for slider, label in zip(self.getLegTeleopSliders(), self.getLegTeleopLabels()):
            label.setText('%.1f' % slider.value())


    def getLegJointPositionsFromSliders(self):
        '''
        returns [hpz, hpx, hpy, kny, aky, akx] in radians
        '''

        hpx = self.ui.hipXSlider.value()
        hpy = self.ui.hipYSlider.value()
        hpz = self.ui.hipZSlider.value()
        kny = self.ui.kneeSlider.value()
        akx = self.ui.ankleXSlider.value()
        aky = self.ui.ankleYSlider.value()

        values = [hpz, hpx, hpy, kny, aky, akx]
        values = [math.radians(x) for x in values]
        return values


    def setLegJointPositionsFromRobotState(self):
        pass

    def onLegTeleop(self):

        autoCommit = self.getAutoCommitIsEnabled()

        data = self.getLegJointPositionsFromSliders()
        data.append(float(autoCommit))
        publishDrivingCommand(lcmdrc.drill_control_t.LEFT_LEG_JOINT_TELEOP, data)


    def onSetLegZeroClicked(self):

        data = self.getLegJointPositionsFromSliders()
        publishDrivingCommand(lcmdrc.drill_control_t.SET_DRIVING_ZERO_POSITION, data)


    def saveSettings(self, settings):
        pass

    def restoreSettings(self, settings):
        pass

    def onCaptureClicked(self):
        pass


class MainWindow(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)
        uic.loadUi(os.path.join(os.path.dirname(__file__), 'DrivingGUI.ui'), self)
        self.setWindowTitle('Driving GUI')
        self.setup()
        self.restoreSettings()

    def setup(self):
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+W'), self), QtCore.SIGNAL('activated()'), self.close)
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+Q'), self), QtCore.SIGNAL('activated()'), self.close)
        self.drivingPanel = DrivingPanel(self)

    def showWarning(self, title, message):
        QtGui.QMessageBox.warning(self, title, message)

    def getSettings(self):
        return QtCore.QSettings('mitdrc', 'DrivingGUI')

    def saveSettings(self):
        settings = self.getSettings()
        self.drivingPanel.saveSettings(settings)

    def restoreSettings(self):
        settings = self.getSettings()
        self.drivingPanel.restoreSettings(settings)

    def closeEvent(self, event):
        self.saveSettings()


def main():


    # create a global instance of the LCMWrapper
    global lcmWrapper
    lcmWrapper = LCMWrapper()

    # start the application
    app = QtGui.QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()
