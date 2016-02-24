import lcm
import drc as lcmdrc
import atlas as lcmatlas
import numpy as np
import time

class AtlasPressureCommander(object):
    def __init__(self,
                 desired_rpm=5000,
                 max_psi=2500,
                 min_psi=1500):
        self.desired_rpm = desired_rpm
        self.max_psi = max_psi
        self.min_psi = min_psi
        self.lc = lcm.LCM()
        self.desired_psi = self.min_psi
        self.last_published_psi = None

    def publish_pump_command(self):
        print "Publishing new desired pump pressure: {:d} PSI".format(self.desired_psi)
        msg = lcmatlas.pump_command_t()
        msg.desired_psi = self.desired_psi
        msg.desired_rpm = self.desired_rpm
        msg.cmd_max = 60.0
        msg.utime = int(time.time() * 1e6)
        self.lc.publish('ATLAS_PUMP_COMMAND', msg.encode())
        self.last_published_psi = self.desired_psi


    def run(self):
        while True:
            time.sleep(0.01)
            self.lc.handle()


class AutoPressureCommander(AtlasPressureCommander):
    def __init__(self,
                 default_offset_psi=300,
                 eta=0.001,
                 publish_threshold_psi=100,
                 **kwargs):
        super(AtlasPressureCommander, self).__init__(**kwargs)
        self.default_offset_psi = default_offset_psi
        self.eta = eta
        self.publish_threshold_psi = publish_threshold_psi
        self._setup_subscriptions()

	def _setup_subscriptions(self):
		self.lc.subscribe('ATLAS_STATE_EXTRA', self.handle_atlas_state_extra)

	def handle_atlas_state_extra(self, channel, msg):
		if isinstance(msg, str):
			msg = lcmatlas.state_extra_t.decode(msg)

		max_joint_psi = max(np.max(msg.psi_pos),
			                np.max(msg.psi_neg))
		self.desired_psi = max(self.desired_psi * (1-self.eta),
			                   max_joint_psi + self.default_offset_psi)

		self.desired_psi = max(self.desired_psi, self.min_psi)
		self.desired_psi = min(self.desired_psi, self.max_psi)

		if self.desired_psi > self.last_published_psi + 5 or self.desired_psi - self.last_published_psi < -self.publish_threshold_psi:
			self.publish_pump_command()


DEFAULT_PLAN_PRESSURE_MAP = {lcmdrc.controller_status_t.MANIPULATING: 2650,
                        lcmdrc.controller_status_t.WALKING: 2000}

DEFAULT_BEHAVIOR_PRESSURE_MAP = {'prep': 1500,
                                 'stand': 2000,
                                 'calibrate_null_bias': 1500,
                                 'calibrate_electric_arms': 1500}


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

if __name__ == '__main__':
	main()
