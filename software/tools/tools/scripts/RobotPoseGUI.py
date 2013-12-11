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
    A basic class provided some convenience methods around LCM.
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


def capturePostureGoal():
    '''
    Blocks until a new LCM message is received on the POSTURE_GOAL channel,
    returns the new message.
    '''
    return lcmWrapper.captureMessage('POSTURE_GOAL', lcmdrc.joint_angles_t)


def capturePoseMessage(captureChannel):
    '''
    A convenience function that aalls the appropriate capture function for the
    given captureChannel.  captureChannel should be EST_ROBOT_STATE or POSTURE_GOAL.
    '''

    captureFunctions = {
      'EST_ROBOT_STATE' : captureRobotState,
      'POSTURE_GOAL' : capturePostureGoal
      }

    assert captureChannel in captureFunctions
    return captureFunctions[captureChannel]()


def getUtime():
    return int(time.time() * 1e6)


def getJointSets():
    '''
    Returns a dictionary of joint sets.
    '''

    leftArm = ['l_arm_usy',
              'l_arm_shx',
              'l_arm_ely',
              'l_arm_elx',
              'l_arm_uwy',
              'l_arm_mwx']

    rightArm = ['r_arm_usy',
              'r_arm_shx',
              'r_arm_ely',
              'r_arm_elx',
              'r_arm_uwy',
              'r_arm_mwx']

    back = [
              'back_bkz',
              'back_bky',
              'back_bkx']

    jointSets = {
                  'left arm' : leftArm,
                  'right arm' : rightArm,
                  'back' : back
                }

    return jointSets


def findPrefixInJointNames(jointNames, prefix):
    for name in jointNames:
        if name.startswith(prefix):
            return True
    return False

def getLeftArmInJoints(joints):
    return findPrefixInJointNames(joints.keys(), 'l_arm')

def getRightArmInJoints(joints):
    return findPrefixInJointNames(joints.keys(), 'r_arm')

def getBackInJoints(joints):
    return findPrefixInJointNames(joints.keys(), 'back')

def getJointNamesForPoseType(poseType):
    '''
    Returns a list of joint names for each part of the robot described in the
    poseType argument.  For example, if poseType is the string 'left arm, right arm',
    then the joint names for the left and right arms is returned.  Supported robot
    parts are left arm, right arm, back.
    '''
    jointSets = getJointSets()
    jointNames = []
    for name, jointSet in jointSets.iteritems():
        if name in poseType:
            jointNames += jointSet
    return jointNames



def updateComboStrings(combo, strings, defaultSelection):
    '''
    Clears the given combobox and then adds the given strings to the combo.
    Restores the combo's current value, or if the combo was empty, uses the
    string given in defaultSelection.
    '''
    currentText = str(combo.currentText()) if combo.count() else defaultSelection
    combo.clear()
    for text in strings:
        if not text:
            combo.insertSeparator(combo.count())
            continue

        combo.addItem(text)
        if text == currentText:
            combo.setCurrentIndex(combo.count() - 1)


def loadConfig(filename):
    '''
    Reads the contents of filename and parses it as a json document, returns
    the result as a dict.
    '''
    assert os.path.isfile(filename)
    with open(filename, 'r') as infile:
        return json.load(infile)


def saveConfig(config, filename):
    '''
    Overwrites the file at filename with a json string generated from the given
    config argument, a dict. 
    '''
    with open(filename, 'w') as outfile:
        json.dump(config, outfile, indent=2, sort_keys=True)


def storePose(poseType, captureChannel, group, name, description, outFile):


    jointSet = getJointNamesForPoseType(poseType)
    assert len(jointSet)

    msg = capturePoseMessage(captureChannel)

    joints = dict()
    for joint, position in zip(msg.joint_name, msg.joint_position):
        if joint in jointSet:
            joints[joint] = position

    posture = dict()
    posture['name'] = name
    posture['description'] = description
    posture['allow_mirror'] = True
    posture['joints'] = joints

    # determine a default value for nominal_handedness
    hasLeft = getLeftArmInJoints(joints)
    hasRight = getRightArmInJoints(joints)
    posture['nominal_handedness'] = 'none'
    if hasLeft != hasRight:
        posture['nominal_handedness'] = 'left' if hasLeft else 'right'

    config = loadConfig(outFile)
    postures = config.setdefault(group, [])

    for existingPosture in postures:
        if existingPosture['name'] == name:
            postures.remove(existingPosture)

    postures.append(posture)
    saveConfig(config, outFile)


def applyMirror(joints):
    '''
    joints is a dict where the keys are joint name strings and the values are
    joint positions.  This function renames l_arm <--> r_arm and flips the sign
    of the joint position as required, and also flips the sign on back_bkz.
    Note that other back joints are not modified by this function.  Returns the
    result as a new dictionary in the same format.
    '''

    def toLeft(jointName):
        return jointName.replace('r_arm', 'l_arm')

    def toRight(jointName):
        return jointName.replace('l_arm', 'r_arm')

    def flipLeftRight(jointName):
          return toLeft(jointName) if jointName.startswith('r_arm') else toRight(jointName)

    signFlips = [
        'l_arm_shx',
        'l_arm_elx',
        'l_arm_mwx',
        'back_bkz'
             ]

    flipped = {}
    for name, position in joints.iteritems():
        name = flipLeftRight(name)
        if toLeft(name) in signFlips:
            position = -position

        flipped[name] = position
    return flipped


def publishPostureGoal(joints, postureName, channel='POSTURE_GOAL'):
    '''
    Given a dict mapping joint name strings to joint positions, creates a
    joint_angles_t LCM message and publishes the result on the given channel name.
    '''
    msg = lcmdrc.joint_angles_t()
    msg.utime = getUtime()
    for name, position in joints.iteritems():
        msg.joint_name.append(name)
        msg.joint_position.append(position)
    msg.num_joints = len(msg.joint_name)
    lcmWrapper.publish(channel, msg)

    # publish a system status message
    msg = lcmdrc.system_status_t()
    msg.utime = getUtime()
    msg.system = 5
    msg.importance = 0
    msg.frequency = 0
    msg.value = 'sending posture goal: ' + postureName
    lcmWrapper.publish('SYSTEM_STATUS', msg)



class SendPosturePanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.selectedPosture = None
        self.setup()


    def setup(self):
        self.ui.postureFilter.hide()
        self.ui.postureFilterLabel.hide()
        self.ui.connect(self.ui.sendLeftButton, QtCore.SIGNAL('clicked()'), self.onLeftClicked)
        self.ui.connect(self.ui.sendRightButton, QtCore.SIGNAL('clicked()'), self.onRightClicked)
        self.ui.connect(self.ui.sendDefaultButton, QtCore.SIGNAL('clicked()'), self.onDefaultClicked)
        self.ui.connect(self.ui.sendPostureGroupCombo, QtCore.SIGNAL('currentIndexChanged(const QString&)'), self.onGroupComboChanged)
        self.ui.connect(self.ui.postureListWidget, QtCore.SIGNAL('currentRowChanged(int)'), self.onPostureSelected)
        self.updateGroupCombo()
        self.updatePostureListWidget()


    def updateGroupCombo(self):
        groupNames = self.ui.getGroupNames()

        try:
            groupNames.remove('General')
        except ValueError:
            pass

        groupNames.insert(0, '')
        groupNames.insert(0, 'General')
        groupNames.insert(0, 'All')
        self.ui.sendPostureGroupCombo.blockSignals(True)
        updateComboStrings(self.ui.sendPostureGroupCombo, groupNames, 'All')
        self.ui.sendPostureGroupCombo.blockSignals(False)

    def setSelectedGroup(self, groupName):
        index = self.ui.sendPostureGroupCombo.findText(groupName)
        if index < 0: index = 0
        self.ui.sendPostureGroupCombo.setCurrentIndex(index)


    def onGroupComboChanged(self):
        self.updatePostureListWidget()

    def getSelectedGroup(self):
        return str(self.ui.sendPostureGroupCombo.currentText())

    def updatePostureListWidget(self):
        groupName = self.getSelectedGroup()
        self.currentPostures = self.ui.getPosturesInGroup(groupName)

        self.ui.postureListWidget.blockSignals(True)
        self.ui.postureListWidget.clear()
        for posture in self.currentPostures:
            self.ui.postureListWidget.addItem(posture['name'])
        self.ui.postureListWidget.setCurrentRow(0)
        self.ui.postureListWidget.blockSignals(False)

        self.onPostureSelected()

    def getSelectedPosture(self):
        currentItem = self.ui.postureListWidget.currentItem()
        if not currentItem:
            return None

        postureName = str(currentItem.text())
        for posture in self.currentPostures:
            if posture['name'] == postureName:
                return posture


    def getPostureCanBeMirrored(self, posture):
        return posture['allow_mirror'] and self.getNominalHandedness(posture) in ('left', 'right')

    def getNominalHandedness(self, posture):
        handedness = posture['nominal_handedness']
        assert handedness in ('left', 'right', 'none')
        return handedness

    def onPostureSelected(self):

        self.selectedPosture = self.getSelectedPosture()
        self.updateDescriptionLabel()

        self.ui.sendDefaultButton.setVisible(False)
        self.ui.sendLeftButton.setVisible(True)
        self.ui.sendRightButton.setVisible(True)
        self.ui.sendDefaultButton.setEnabled(False)
        self.ui.sendLeftButton.setEnabled(False)
        self.ui.sendRightButton.setEnabled(False)
        if not self.selectedPosture:
            return

        if self.getPostureCanBeMirrored(self.selectedPosture):
            self.ui.sendLeftButton.setEnabled(True)
            self.ui.sendRightButton.setEnabled(True)
        else:
            self.ui.sendLeftButton.setVisible(False)
            self.ui.sendRightButton.setVisible(False)
            self.ui.sendDefaultButton.setVisible(True)
            self.ui.sendDefaultButton.setEnabled(True)


    def updateDescriptionLabel(self):
        description = self.selectedPosture['description'] if self.selectedPosture else 'none'
        self.ui.descriptionLabel.setText('Description: ' + str(description))


    def onGroupsChanged(self):
        self.updateGroupCombo()

    def onPostureAdded():
        self.updatePostureListWidget()

    def onLeftClicked(self):
        joints = self.selectedPosture['joints']
        if self.getNominalHandedness(self.selectedPosture) == 'right':
            joints = applyMirror(joints)
        publishPostureGoal(joints, self.selectedPosture['name'] + ' left')

    def onRightClicked(self):
        joints = self.selectedPosture['joints']
        if self.getNominalHandedness(self.selectedPosture) == 'left':
            joints = applyMirror(joints)
        publishPostureGoal(joints, self.selectedPosture['name'] + ' right')

    def onDefaultClicked(self):
        joints = self.selectedPosture['joints']
        publishPostureGoal(joints, self.selectedPosture['name'] + ' default')

    def saveSettings(self, settings):
        settings.setValue('sendPose/currentGroup', self.getSelectedGroup())

    def restoreSettings(self, settings):
        self.setSelectedGroup(settings.value('sendPose/currentGroup', 'All').toString())


class CapturePanel(object):

    def __init__(self, ui):
        self.ui = ui
        self.setup()


    def setup(self):
        self.ui.connect(self.ui.captureButton, QtCore.SIGNAL('clicked()'), self.onCaptureClicked)
        self.ui.connect(self.ui.groupCombo, QtCore.SIGNAL('currentIndexChanged(const QString&)'), self.onGroupComboChanged)
        self.updateGroupCombo()


    def updateGroupCombo(self):
        groupNames = self.ui.getGroupNames()
        try:
            groupNames.remove('General')
        except ValueError:
            pass

        groupNames.insert(0, '')
        groupNames.insert(0, 'General')
        groupNames.append('')
        groupNames.append('New group...')
        self.ui.groupCombo.blockSignals(True)
        updateComboStrings(self.ui.groupCombo, groupNames, 'General')
        self.ui.groupCombo.blockSignals(False)

    def setSelectedGroup(self, groupName):
        index = self.ui.groupCombo.findText(groupName)
        if index < 0: index = 0
        self.ui.groupCombo.setCurrentIndex(index)

    def getSelectedGroup(self):
        return str(self.ui.groupCombo.currentText())

    def onGroupComboChanged(self):

        if str(self.ui.groupCombo.currentText()) == 'New group...':
            groupName, ok = QtGui.QInputDialog.getText(self.ui, 'Enter new group name', 'Group name:')
            if not ok or not groupName:
                self.setSelectedGroup('General')
                return

            groupName = str(groupName)

            self.ui.addNewGroup(groupName)
            self.setSelectedGroup(groupName)

    def onGroupsChanged(self):
        self.updateGroupCombo()

    def saveSettings(self, settings):
        settings.setValue('sendPose/currentGroup', self.getSelectedGroup())

    def restoreSettings(self, settings):
        self.setSelectedGroup(settings.value('capturePanel/currentGroup', 'General').toString())

    def onCaptureClicked(self):

        captureChannel = str(self.ui.captureChannelCombo.currentText())
        group = str(self.ui.groupCombo.currentText())
        name = str(self.ui.nameEdit.text())
        description = str(self.ui.descriptionEdit.text())
        poseType = str(self.ui.jointSetCombo.currentText())
        outFile = self.ui.getPoseConfigFile()

        if not name:
            self.ui.showWarning('Empty field', 'Please enter a name into the text box.')
            return

        if not description:
            self.ui.showWarning('Empty field', 'Please enter a description into the text box.')
            return

        existingPostures = self.ui.getPosturesInGroup(group)
        for posture in existingPostures:
            if posture['name'] == name:
                reply = QtGui.QMessageBox.question(self.ui, 'Overwrite posture?', 'Posture with name "%s" already exists.\nDo you want to overwrite?' % name,
                                                  QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)

                if reply == QtGui.QMessageBox.No:
                    return

        storePose(poseType, captureChannel, group, name, description, outFile)
        self.ui.onPostureAdded()


class MainWindow(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)
        uic.loadUi(os.path.join(os.path.dirname(__file__), 'RobotPoseGUI.ui'), self)
        self.setWindowTitle('Robot Pose Utility')
        if not self.checkConfigFile():
            return

        self.setup()
        self.restoreSettings()

    def setup(self):
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+W'), self), QtCore.SIGNAL('activated()'), self.close)
        self.connect(QtGui.QShortcut(QtGui.QKeySequence('Ctrl+Q'), self), QtCore.SIGNAL('activated()'), self.close)
        self.capturePanel = CapturePanel(self)
        self.sendPosturePanel = SendPosturePanel(self)

    def showWarning(self, title, message):
        QtGui.QMessageBox.warning(self, title, message)

    def getSettings(self):
        return QtCore.QSettings('mitdrc', 'RobotPoseGUI')

    def saveSettings(self):
        settings = self.getSettings()
        settings.setValue('currentTabIndex', self.tabWidget.currentIndex())
        self.capturePanel.saveSettings(settings)
        self.sendPosturePanel.saveSettings(settings)

    def restoreSettings(self):
        settings = self.getSettings()
        self.tabWidget.setCurrentIndex(settings.value('currentTabIndex', 0).toInt()[0])
        self.capturePanel.restoreSettings(settings)
        self.sendPosturePanel.restoreSettings(settings)


    def closeEvent(self, event):
        self.saveSettings()


    def checkConfigFile(self):
        if not os.path.isdir(os.environ['DRC_BASE']):
            self.showWarning('Environment not set', 'Error loading configuration.  DRC_BASE environment variable is not define to a valid directory.')
            self.setEnabled(False)
            return False

        configFile = self.getPoseConfigFile()
        if not os.path.isfile(configFile):
            self.showWarning('Config file not found', 'Config file not found: %s' % configFile)
            self.setEnabled(False)
            return False

        try:
            json.load(open(configFile, 'r'))
        except ValueError as exc:
            self.showWarning('Parse error', 'Error parsing json file: %s' % str(exc))
            self.setEnabled(False)
            return False

        return True

    def getPoseConfigFile(self):
        return os.path.join(os.environ['DRC_BASE'], 'software/config/task_config/stored_poses.json')

    def loadConfigFile(self):
        if not self.checkConfigFile():
            return
        config = json.load(open(self.getPoseConfigFile(), 'r'))
        if not self.checkPostures(config):
            return {}
        return config

    def getGroupNames(self):
        if not self.checkConfigFile():
            return []
        return sorted(self.loadConfigFile().keys())


    def checkPostures(self, config):
        for groupName, postures in config.iteritems():
            for i, posture in enumerate(postures):
                for name in ['name', 'description', 'joints', 'nominal_handedness']:
                    if name not in posture:
                        self.ui.showWarning('Format error', 'Format error in posture %d of group "%s".  Missing attribute: "%s".' % (i, groupName, name))
                        self.currentConfig = {}
                        return False
        return True


    def getPosturesInGroup(self, groupName):

        config = self.loadConfigFile()

        postures = []
        if groupName == 'All':
            for group, postureList in config.iteritems():
                for posture in postureList:
                    posture['name'] = '%s - %s' % (group, posture['name'])
                postures += postureList
        else:
            postures = config.get(groupName, [])

        return sorted(postures, key=lambda x: x['name'])


    def addNewGroup(self, groupName):
        config = self.loadConfigFile()
        config.setdefault(groupName, [])
        saveConfig(config, self.getPoseConfigFile())

        self.capturePanel.onGroupsChanged()
        self.sendPosturePanel.onGroupsChanged()

    def onPostureAdded(self):
        self.sendPosturePanel.updatePostureListWidget()




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
