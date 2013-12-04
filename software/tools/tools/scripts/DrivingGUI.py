#!/usr/bin/env python

from PyQt4 import QtCore, QtGui, uic

import lcm
import sys
import json
import time
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
        self.ui.connect(self.ui.throttleSlider, QtCore.SIGNAL('valueChanged(int)'), self.onThrottleSliderValueChanged)
        self.ui.connect(self.ui.steeringSlider, QtCore.SIGNAL('valueChanged(int)'), self.onSteeringSliderValueChanged)

        self.ui.connect(self.ui.setSteeringDepthButton, QtCore.SIGNAL('clicked()'), self.onSetSteeringDepthClicked)
        self.ui.connect(self.ui.refitSteeringWheelButton, QtCore.SIGNAL('clicked()'), self.onRefitSteeringWheelClicked)

        self.onThrottleSliderValueChanged()
        self.onSteeringSliderValueChanged()

    def getSteeringValue(self):
        return self.ui.steeringSlider.value() / 100.0

    def getThrottleValue(self):
        return self.ui.throttleSlider.value() / 100.0

    def onSteeringSliderValueChanged(self):
        self.ui.steeringLabel.setText(str(self.getSteeringValue()))
        self.publishDrivingControl()

    def onThrottleSliderValueChanged(self):
        self.ui.throttleLabel.setText(str(self.getThrottleValue()))
        self.publishDrivingControl()

    def onRefitSteeringWheelClicked(self):
        publishDrivingCommand(lcmdrc.drill_control_t.REFIT_STEERING, [])

    def onSetSteeringDepthClicked(self):
        steeringDepth = self.ui.steeringDepthSpin.value() / 100.0
        publishDrivingCommand(lcmdrc.drill_control_t.SET_STEERING_DEPTH, [steeringDepth])

    def publishDrivingControl(self):
        publishDrivingCommand(lcmdrc.drill_control_t.DRIVING_CONTROL, [self.getSteeringValue(), self.getThrottleValue()])




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
