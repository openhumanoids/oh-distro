import lcm
import drc as lcmdrc
import datetime
import select

class LCMPoller(object):

    def __init__(self, lc):
        self.lc = lc
        self.poll = select.poll()
        self.poll.register(self.lc.fileno())

    def handleLCM(self, timeout=100):
        if self.poll.poll(timeout):
            self.lc.handle()


def getTimeString():
    return datetime.datetime.now().strftime('%H:%M:%S')


planMsgCount = 0


def onCommittedRobotPlan(channel, msgBytes):

    global planMsgCount
    msg = lcmdrc.robot_plan_t.decode(msgBytes)
    print 'received message %d on %s with utime %r at: %s' % (planMsgCount, channel, msg.utime, getTimeString())
    planMsgCount += 1


confMsgCount = 0

def onConfigurationTraj(channel, msgBytes):
    global confMsgCount
    print 'received message %d on %s at: %s' % (confMsgCount, channel, getTimeString())
    confMsgCount += 1

def main():

    lc = lcm.LCM()
    poller = LCMPoller(lc)


    lc.subscribe('COMMITTED_ROBOT_PLAN', onCommittedRobotPlan)
    lc.subscribe('CONFIGURATION_TRAJ', onConfigurationTraj)

    while True:
        poller.handleLCM()


if __name__ == '__main__':
    main()


class PlanPressureCommander(AtlasPressureCommander):
    def __init__(self,
                 plan_map=DEFAULT_PLAN_PRESSURE_MAP,
                 behavior_map=DEFAULT_BEHAVIOR_PRESSURE_MAP,
                 **kwargs):
        super(PlanPressureCommander, self).__init__(**kwargs)
        self.plan_map = plan_map
        self.behavior_map = behavior_map
        self._setup_subscriptions()

    def _setup_subscriptions(self):
        self.lc.subscribe('CONTROLLER_STATUS', self.handle_controller_status)
        self.lc.subscribe('ATLAS_BEHAVIOR_COMMAND', self.handle_behavior)

    def handle_controller_status(self, channel, msg):
        if isinstance(msg, str):
            msg = lcmdrc.controller_status_t.decode(msg)

        if msg.state in self.plan_map:
            self.desired_psi = self.plan_map[msg.state]
            self.publish_pump_command()

    def handle_behavior(self, channel, msg):
        if isinstance(msg, str):
            msg = lcmdrc.behavior_command_t.decode(msg)

        s = msg.command.lower()
        if s in self.behavior_map:
            self.desired_psi = self.behavior_map[s]
            self.publish_pump_command()

    def run(self):
        while True:
            time.sleep(0.1)
            self.lc.handle()

def main():
    mon = PlanPressureCommander()
    print "Pressure Command: ready"
    mon.run()

