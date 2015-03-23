import lcm
import drc as lcmdrc
import numpy as np
import time

class AtlasPressureMonitor(object):
	def __init__(self, 
		         default_offset_psi=300, 
		         eta=0.001,
		         desired_rpm=5000,
		         max_psi=2500,
		         min_psi=1500,
		         publish_threshold_psi=100):
		self.default_offset_psi = default_offset_psi
		self.eta = eta
		self.desired_psi = 0
		self.desired_rpm = desired_rpm
		self.lc = lcm.LCM()
		self._setup_subscriptions()
		self.max_psi = max_psi
		self.min_psi = min_psi
		self.last_published_psi = 0
		self.publish_threshold_psi = publish_threshold_psi

	def _setup_subscriptions(self):
		self.lc.subscribe('ATLAS_STATE_EXTRA', self.handle_atlas_state_extra)

	def handle_atlas_state_extra(self, channel, msg):
		if isinstance(msg, str):
			msg = lcmdrc.atlas_state_extra_t.decode(msg)

		max_joint_psi = max(np.max(msg.psi_pos),
			                np.max(msg.psi_neg))
		self.desired_psi = max(self.desired_psi - 1,
			                   max_joint_psi + self.default_offset_psi)

		self.desired_psi = max(self.desired_psi, self.min_psi)
		self.desired_psi = min(self.desired_psi, self.max_psi)

		if self.desired_psi > self.last_published_psi + 5 or self.desired_psi - self.last_published_psi < -self.publish_threshold_psi:
			self.publish_pump_command()

	def publish_pump_command(self):
		msg = lcmdrc.atlas_pump_command_t()
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


def main():
	mon = AtlasPressureMonitor()
	mon.run()

if __name__ == '__main__':
	main()