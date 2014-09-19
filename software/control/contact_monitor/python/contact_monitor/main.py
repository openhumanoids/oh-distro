import lcm, os, sys, drc, math, time

"""
A simple foot contact classifier for Atlas.

"""

lc = lcm.LCM()

class ContactMonitor:
  def __init__(self,fz_threshold,channel):
    self.fz_threshold = fz_threshold # newtons
    self.l_foot_contact = False
    self.r_foot_contact = False
    self.last_left_change_t = 0;
    self.last_right_change_t = 0;
    self.debounce_time = 0.05 # seconds
      # note: could also require force be greater than X for at least t secs
    self.channel = channel

  def state_handle(self, channel, data):
    msg = drc.robot_state_t.decode(data)

    if (msg.utime-self.last_left_change_t > self.debounce_time*1e6):
      self.l_foot_contact = msg.force_torque.l_foot_force_z > self.fz_threshold
      self.last_left_change_t = msg.utime
    if (msg.utime-self.last_right_change_t > self.debounce_time*1e6):
      self.r_foot_contact = msg.force_torque.r_foot_force_z > self.fz_threshold
      self.last_right_change_t = msg.utime

    output = drc.foot_contact_estimate_t()
    output.utime = msg.utime
    output.detection_method = 0;
    output.left_contact = 1*self.l_foot_contact
    output.right_contact = 1*self.r_foot_contact
    lc.publish(self.channel, output.encode())

  
def main():

  if len(sys.argv) > 1:
    thresold = float(sys.argv[1]) # newtons
  else:
    thresold = 150 # newtons
  
  if len(sys.argv) > 2:
    channel = sys.argv[2]
  else:
    channel  = "FOOT_CONTACT_ESTIMATE"

  c = ContactMonitor(thresold,channel)
  lc.subscribe("EST_ROBOT_STATE", c.state_handle)
  print 'running contact monitor...'
  while True:
    lc.handle()

if __name__ == '__main__':
  main()
