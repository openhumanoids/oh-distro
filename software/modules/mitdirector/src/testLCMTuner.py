import PythonQt
from PythonQt import QtCore, QtGui
from director.timercallback import TimerCallback
import director.objectmodel as om
from director import propertyset
import director.applogic as app

import LCMTuner


def startApplication(enableQuitTimer=False):
    appInstance = QtGui.QApplication.instance()
    if enableQuitTimer:
        quitTimer = TimerCallback()
        quitTimer.callback = appInstance.quit
        quitTimer.singleShot(0.1)
    appInstance.exec_()


def main():
    globalsDict = globals()
    lcmTunerManager = LCMTuner.LCMTunerManager()
    globalsDict['lcmTunerManager'] = lcmTunerManager
    _pythonManager.consoleWidget().show()
    startApplication(enableQuitTimer=False)


if __name__ == '__main__':
    main()
