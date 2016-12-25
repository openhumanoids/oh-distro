import drc as lcmdrc
from director import lcmUtils
import director.objectmodel as om
import PythonQt
from PythonQt import QtCore, QtGui
from director import propertyset


PUBLISH_CHANNEL = "LCM_TUNER_SET_VALUES"

class LCMTuner(object):

    def __init__(self, status_msg):
        self.initial_status_msg = status_msg
        self.name = str(status_msg.tuner_name)
        self.omItem = om.ObjectModelItem(self.name)
        self.varNames = set() # dict to store the names of tuned variables
        self.addPropertiesToObjectModelItem(status_msg)
        self.omItem.properties.connectPropertyChanged(self.onPropertyChanged)

        # launch the panel
        self.panel = PythonQt.dd.ddPropertiesPanel()
        self.panel.setBrowserModeToWidget()
        self.panel.show()
        self.panel.setWindowTitle(self.name)
        self.panelConnector = propertyset.PropertyPanelConnector(self.omItem.properties, self.panel)

    # status msg is of type lcm_tuner_status_t.lcm
    def addPropertiesToObjectModelItem(self, status_msg):
        for idx, var_name in enumerate(status_msg.var_names):
            var_name = str(var_name) # make sure we aren't getting some weird unicode thing
            current_val = status_msg.current_val[idx]
            min_val = status_msg.min_val[idx]
            max_val = status_msg.max_val[idx]

            # check if this property already exists or not
            if not self.omItem.hasProperty(var_name):
                self.varNames.add(var_name)
                self.omItem.addProperty(var_name, current_val,
                                        attributes=om.PropertyAttributes(decimals=2, minimum=min_val,
                                                                     maximum=max_val, singleStep=0.1))
    def onPropertyChanged(self, propertySet, propertyName):
        self.sendLCMTunerSetterMsg()

    def onStatusMsg(self, status_msg):
        self.addPropertiesToObjectModelItem(status_msg)

    # publish a lcm_tuner_setter_t msg with all the current values
    def sendLCMTunerSetterMsg(self):
        msg = lcmdrc.lcm_tuner_setter_t()
        msg.tuner_name = self.name
        var_names = []
        values = []

        for var_name in self.varNames:
            var_names.append(var_name)
            values.append(self.omItem.getProperty(var_name))

        msg.num_vars = len(var_names)
        msg.var_names = var_names
        msg.values = values

        lcmUtils.publish(PUBLISH_CHANNEL, msg)

    def showPanel(self):
        self.panel.show()

    def hidePanel(self):
        self.panel.hide()



    # def setupSubscribers(self):
    #     lcmUtils.addSubscriber("LCM_TUNER_STATUS", lcmdrc.lcm_tuner_status_t, self.testReceiveStatusMessage)
    #
    # def testReceiveStatusMessage(self, msg):
    #     self.testTunerName = str(msg.tuner_name)
    #
    # def sendTestMessage(self, value=0.5):
    #     msg = lcmdrc.lcm_tuner_setter_t()
    #     msg.tuner_name = self.testTunerName
    #     msg.num_vars = 1
    #     msg.var_names = ["test_var"]
    #     msg.values = [value]
    #
    #     lcmUtils.publish("LCM_TUNER_SET_VALUES", msg)


class LCMTunerManager(object):

    def __init__(self):
        self.lcmTunerDict = dict()
        self.setupSubscribers()

    def setupSubscribers(self):
        lcmUtils.addSubscriber("LCM_TUNER_STATUS", lcmdrc.lcm_tuner_status_t, self.onLCMTunerStatus)

    def onLCMTunerStatus(self, msg):
        tuner_name = str(msg.tuner_name)
        if tuner_name in self.lcmTunerDict:
            # pass it through to the correct lcm tuner if it exists
            self.lcmTunerDict[tuner_name].onStatusMsg(msg)
        else: # otherwise create a new lcm tuner object and add it to the dict
            lcmTuner = LCMTuner(msg)
            self.lcmTunerDict[tuner_name] = lcmTuner

