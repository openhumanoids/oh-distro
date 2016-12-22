import drc as lcmdrc
from director import lcmUtils


class LCMTuner:

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.setupSubscribers()

    def setupSubscribers(self):
        lcmUtils.addSubscriber("LCM_TUNER_STATUS", lcmdrc.lcm_tuner_status_t, self.testReceiveStatusMessage)

    def testReceiveStatusMessage(self, msg):
        self.testTunerName = str(msg.tuner_name)

    def sendTestMessage(self, value=0.5):
        msg = lcmdrc.lcm_tuner_setter_t()
        msg.tuner_name = self.testTunerName
        msg.num_vars = 1
        msg.var_names = ["test_var"]
        msg.values = [value]

        lcmUtils.publish("LCM_TUNER_SET_VALUES", msg)
