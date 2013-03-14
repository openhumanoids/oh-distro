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

home_dir =os.getenv("HOME")
#print home_dir
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.bandwidth_stats_t import bandwidth_stats_t
########################################################################################

def timestamp_now (): return int (time.time () * 1000000)

class SensorData(object):
    def __init__(self, nfields):
        self.nfields = nfields
        self.reset()
    def append(self, utime,v_in ):
        #np_utimes = np.array(utime)
        np_utimes = np.array( (utime - first_utime)/1000000.0 )
        self.utimes = np.vstack((self.utimes , np_utimes ))
        np_v = np.array(v_in)
        self.v = np.vstack((self.v , np_v ))
    def reset(self):
        # no sure how to support initialising a Nx0 array, so I'm initing a Nx1 array and skipping 1st row:
        self.utimes=np.array([0]) 
        self.v=np.zeros((1, self.nfields))

        
def reset_all():
  msgs.reset(); bytes.reset(); cumbytes.reset()

  
def plot_data():
  global last_utime
  global channels
  front_block =0 # offset into the future (to ensure no recent data not viewed)
  print "len of channels: %d" %  len(channels)
  if ( len(msgs.utimes) >1):
    plt.figure(1)
    print "draw"
    ############################################################
    ax1.cla()
    cols = 'rgbkmcwrgbkmcwrgbkmcwrgbkmcwrgbkmcwrgbkmcw'
    for i in range(len(channels)): 
      if(msgs.v[-1,i]>0):
        ax1.plot(msgs.utimes[1:], np.transpose(msgs.v[1:,i]),color=cols[i] , linewidth=1,label=channels[i])
    ax1.set_xlabel('Time [simsec] (sim utime: '+ str(last_utime) +')');  ax1.set_ylabel('N. messages queued');  ax1.grid(True)
    ax1.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )

    ax2.cla()
    for i in range(len(channels)): 
      if(msgs.v[-1,i]>0):
        ax2.plot(bytes.utimes[1:], np.transpose(bytes.v[1:,i]),color=cols[i] ,  linewidth=1,label=channels[i])
    ax2.set_xlabel('Time [simsec] (sim utime: '+ str(last_utime) + ')');  ax2.set_ylabel('N. kBytes [1024 bits] queued');  ax2.grid(True)
    ax2.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax2.set_ylim( bottom=0)
    
    ax3.cla()
    for i in range(len(channels)): 
      if(msgs.v[-1,i]>0):
        ax3.plot(cumbytes.utimes[1:], np.transpose(cumbytes.v[1:,i]),color=cols[i] ,  linewidth=1,label=channels[i])

    ax3.set_xlabel('Time [simsec] (sim utime: '+ str(last_utime) + ')');  ax3.set_ylabel('Cum. N. kBytes [1024 bits]');  ax3.grid(True)
    ax3.legend(loc=2,prop={'size':14})
    ax3.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax3.set_ylim( bottom=0)

    ############################################################
    ax4.cla()
    j=-1
    for i in range(0,len(channels)): 
      if(msgs.v[-1,i]>0):
        if(j==-1):
          ax4.fill_between( cumbytes.utimes[1:,0][:], 0 , cumbytes.v[1:,i][:],color=cols[i] , label=channels[i]) 
        else:
          ax4.fill_between( cumbytes.utimes[1:,0][:], cumbytes.v[1:,j][:] , cumbytes.v[1:,i][:],color=cols[i] , label=channels[i]) 
        j=i
        ax4.plot(cumbytes.utimes[1:], np.transpose(cumbytes.v[1:,i]),color=cols[i] ,  linewidth=1,label=channels[i])
    ax4.set_xlabel('Time [simsec] (sim utime: '+ str(last_utime) + ')');  ax4.set_ylabel('Cum. N. kBytes [1024]');  ax4.grid(True)
    ax4.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax4.set_ylim( bottom=0)
    

    
  plt.plot()
  plt.draw()
  

# Microstrain INS/IMU Sensor:
def on_bw(channel, data):
  print "SDfsdf"
  m = bandwidth_stats_t.decode(data)
  
  # this keys of first recieved time
  #global first_utime
  #if (first_utime==0):
  #  first_utime= m.utime
  
  this_bytes=[]
  for i in range(len(m.queued_bytes)): 
    this_bytes.append(m.queued_bytes[i]/1024.0)
  this_cumbytes = np.cumsum(this_bytes)
  msgs.append(m.utime,m.queued_msgs)
  bytes.append(m.utime,this_bytes)
  cumbytes.append(m.utime,this_cumbytes)
  
  global channels
  channels = m.channels
  global last_utime
  if (m.utime < last_utime):
    print "out of order data, resetting now %s | last %s"   %(m.utime,last_utime)
    reset_all()
  last_utime = m.utime
  
#################################################################################
channels=[]

lc = lcm.LCM()
print "started"
last_utime=0
first_utime=0
plot_window=30*1000000 #3sec
msgs = SensorData(17); bytes = SensorData(17);
cumbytes = SensorData(17);

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
  sub1 = lc.subscribe("BW_STATS", on_bw) # required
  
  while True:
    ## Handle LCM if new messages have arrived.
    lc.handle()

  lc.unsubscribe(sub1)

t2 = Thread(target=lcm_thread)
t2.start()

time.sleep(3) # wait for some data- could easily remove

plot_timing=0.1 # time between updates of the plots - in wall time
while (1==1):
  time.sleep(plot_timing)
  tic_ms = float(round(time.time() * 1000))
  plot_data()
  toc_ms = float(round(time.time() * 1000))
  dt_sec = (toc_ms - tic_ms)/1000
  print "drawing time: %f" %(dt_sec)
