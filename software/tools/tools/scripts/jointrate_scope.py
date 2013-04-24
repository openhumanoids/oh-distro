#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

from threading import Thread
import threading

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.joint_command_t import joint_command_t
from drc.utime_t import utime_t
########################################################################################
def timestamp_now (): return int (time.time () * 1000000)


class SensorData(object):
    def __init__(self, nfields):
        self.nfields = nfields
        self.reset()
    def append(self, utime,v_in ):
        np_v = np.array(v_in)
        # reset if shape is different
        if (self.v.shape[1] != np_v.size):
          self.nfields = np_v.size
          self.reset()
        np_utimes = np.array( (utime - s.first_utime)/1000000.0 )
        self.utimes = np.vstack((self.utimes , np_utimes ))
        self.v = np.vstack((self.v , np_v ))
    def reset(self):
        # no sure how to support initialising a Nx0 array, so I'm initing a Nx1 array and skipping 1st row:
        self.utimes=np.array([0]) 
        self.v=np.zeros((1, self.nfields))


def plot_data():
  front_block =0 # offset into the future (to ensure no recent data not viewed)
  if ( len(controlrate.utimes) >1):
    plt.figure(1)
    ax1.cla()
    ax1.plot( controlrate.utimes[2:], np.transpose(controlrate.v[2:,0]), 'r', linewidth=1,label="control rate [Hz]")
    ax1.legend();   ax1.set_xlabel('Time - sec | ' + str(len(controlrate.utimes)));   ax1.set_ylabel('Control Rate [Hz]');   ax1.grid(True)
    ax1.legend(loc=2,prop={'size':10})
    #ax1.set_xlim( (s.last_utime - s.plot_window - s.first_utime)/1000000 , (s.last_utime + front_block - s.first_utime)/1000000 )

  plt.plot()
  plt.draw()

class State:
  def __init__(self, counter, prev_utime):
    self.counter = 0
    self.prev_utime = 0
    self.prev_counter_utime=0
    self.period =0.1*1E6 # averaging period in sec
    #plotting variable
    self.last_utime=0
    self.first_utime=0
    self.plot_window=3*1000000 #3sec

global s
s = State(0, 0)

# Gazebo Simulated IMU:
def on_joint_commands(channel, data):
  m = joint_command_t.decode(data)
  s.counter=s.counter+1
  #print "got %d" %(s.counter)

def on_utime(channel, data):
  m = utime_t.decode(data)
  #print "======="
  rem = round(m.utime/s.period)
  prev_rem = round(s.prev_utime/s.period)
  if (rem!=prev_rem):
    this_controlrate= float( 1000000.0*s.counter/(m.utime - s.prev_counter_utime) )
    print "%d commands in %f sim sec | %fHz" %(s.counter, s.period/1E6, this_controlrate)
    controlrate.append(m.utime,this_controlrate)
    s.prev_counter_utime = m.utime
    s.counter=0
  #print "%d === %d" % (prev_rem, rem)
  #print "%d === %d" % (s.prev_utime, m.utime)
  if (s.first_utime==0):
    s.first_utime= m.utime
  s.prev_utime = m.utime
  s.last_utime = m.utime  

####################################################################
lc = lcm.LCM()
print "started"
controlrate = SensorData(1);


left, bottom, width, height =0.07, 0.07, 0.395, 0.395
box_ul = [left, 2*bottom+height, width, height]
box_ur = [2*left+width, 2*bottom+height, width, height]
box_ll = [left, bottom, width, height]
box_lr = [2*left+width, bottom, width, height]



fig1 = plt.figure(num=1, figsize=(14, 10), dpi=80, facecolor='w', edgecolor='k')
ax1 = fig1.add_axes(box_ul)
ax2 = fig1.add_axes(box_ur)
ax3 = fig1.add_axes(box_ll)
ax4 = fig1.add_axes(box_lr)

plt.interactive(True)
plt.plot()
plt.draw()

def lcm_thread():
  sub = lc.subscribe("JOINT_COMMANDS", on_joint_commands)
  sub1 = lc.subscribe("ROBOT_UTIME", on_utime)

  while True:
    ## Handle LCM if new messages have arrived.
    lc.handle()

  lc.unsubscribe(sub)

####################################################################
t2 = Thread(target=lcm_thread)
t2.start()

lock = threading.Lock()


#time.sleep(3) # wait for some data- could easily remove

plot_timing=0.1 # time between updates of the plots - in wall time
while (1==1):
  time.sleep(plot_timing)
  tic_ms = float(round(time.time() * 1000))
  lock.acquire()
  plot_data()
  lock.release()
  toc_ms = float(round(time.time() * 1000))
  dt_sec = (toc_ms - tic_ms)/1000
  #print "drawing time: %f" %(dt_sec)




