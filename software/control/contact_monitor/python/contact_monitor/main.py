import lcm, os, sys, drc, math, time

"""
A simple foot contact classifier for Atlas.

"""

lc = lcm.LCM()

class ContactMonitor:
    def __init__(self):
		self.fz_threshold = 200 # newtons
		self.l_foot_contact = False
		self.r_foot_contact = False

    def state_handle(self, channel, data):
		msg = drc.robot_state_t.decode(data)

		self.l_foot_contact = msg.force_torque.l_foot_force_z > self.fz_threshold
		self.r_foot_contact = msg.force_torque.r_foot_force_z > self.fz_threshold

		print msg.force_torque.r_foot_force_z
		output = drc.foot_contact_estimate_t()
		output.utime = msg.utime
		output.detection_method = 0;
		output.left_contact = 1*self.l_foot_contact
		output.right_contact = 1*self.r_foot_contact
		lc.publish("FOOT_CONTACT_ESTIMATE", output.encode())

  
def main():
	c = ContactMonitor()
	lc.subscribe("EST_ROBOT_STATE", c.state_handle)
	while True:
		lc.handle()

if __name__ == '__main__':
	main()
