#!/usr/bin/env python
import rosbag
bag = rosbag.Bag('finger_0.bag')
for topic, msg, t in bag.read_messages(topics=['raw_finger_status_0']):
  numbers = [item for sublist in [msg.hall_tgt, msg.hall_pos, msg.fmcb_effort] for item in sublist]
  print " ".join(str(d) for d in numbers)
bag.close()
