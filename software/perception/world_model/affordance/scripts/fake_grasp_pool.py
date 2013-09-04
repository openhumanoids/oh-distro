#!/usr/bin/python
import os,sys
home_dir =os.getenv("DRC_BASE")
#print home_dir
sys.path.append(home_dir + "/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/software/build/lib/python2.7/dist-packages")

import lcm
from drc.grasp_opt_status_t import grasp_opt_status_t
import time

msg = grasp_opt_status_t()
msg.utime = 0
msg.matlab_pool_ready=1
msg.num_matlab_workers=2
msg.worker_available=1

lc = lcm.LCM()
while (1):
  

  msg.worker_id =1
  lc.publish("GRASP_OPT_STATUS", msg.encode())

  msg.worker_id =2
  lc.publish("GRASP_OPT_STATUS", msg.encode())
  print "GRASP_OPT_STATUS sent"
  time.sleep(1)  


