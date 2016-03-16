import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
import os

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
