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
        np_v = np.array(v_in)
        if (self.v.shape[1] != np_v.size):
          self.nfields = np_v.size
          self.reset()
        np_utimes = np.array( (utime - first_utime)/1000000.0 )
        self.times = np.vstack((self.times , np_utimes ))
        self.v = np.vstack((self.v , np_v ))
    def reset(self):
        # no sure how to support initialising a Nx0 array, so I'm initing a Nx1 array and skipping 1st row:
        self.times=np.array([0]) 
        self.v=np.zeros((1, self.nfields))

        
def reset_all():
  msgs.reset(); queued_KB.reset(); cumqueued_KB.reset(); cumsent_KB.reset(); cumreceived_KB.reset(); ratesent_KB.reset(); ratereceived_KB.reset();

  
def plot_data():
  global last_utime
  global sent_channels, got_rate
  front_block =0 # offset into the future (to ensure no recent data not viewed)
  #print "len of sent_channels: %d" %  len(sent_channels)
  cols = 'bgrcykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmyk'
  if ( len(msgs.times) >1):
    plt.figure(1)
    ############################################################
    ax1.cla()
    for i in range(0,len(sent_channels)-1): 
      if(i==0):
        ax1.fill_between( cumsent_KB.times[1:,0][:], 0 , cumsent_KB.v[1:,i][:],color=cols[i] , label=sent_channels[i]) 
      else:
        ax1.fill_between( cumsent_KB.times[1:,0][:], cumsent_KB.v[1:,i-1][:] , cumsent_KB.v[1:,i][:],color=cols[i] , label=sent_channels[i]) 
      ax1.plot(cumsent_KB.times[1:], np.transpose(cumsent_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=sent_channels[i])
    
    ax1.set_ylabel('Cum KB Sent [' + msg_channel +']');  ax2.grid(True)
    ax1.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax1.set_ylim( bottom=0)
    ax1.legend(loc=2,prop={'size':10})
    
    ############################################################
    ax2.cla()
    for i in range(0,len(sent_channels)-1): 
      if(i==0):
        ax2.fill_between( cumsent_KB.times[1:,0][:], 0 , cumsent_KB.v[1:,i][:],color=cols[i] , label=sent_channels[i]) 
      else:
        ax2.fill_between( cumsent_KB.times[1:,0][:], cumsent_KB.v[1:,i-1][:] , cumsent_KB.v[1:,i][:],color=cols[i] , label=sent_channels[i]) 
      ax2.plot(cumsent_KB.times[1:], np.transpose(cumsent_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=sent_channels[i])
    
    one_strip = np.ones( ( len(cumsent_KB.times[1:,0][:]),1) )
    ax2.plot(cumsent_KB.times[1:,0][:],  np.multiply( cumsent_KB.times[1:,0][:] , 4 ),'r:',linewidth=2 )
    ax2.plot(cumsent_KB.times[1:,0][:],  np.multiply( cumsent_KB.times[1:,0][:] , 1 ),'y:',linewidth=2 )
    ax2.plot(cumsent_KB.times[1:,0][:],  np.multiply( cumsent_KB.times[1:,0][:] , 0.25 ),'g:',linewidth=2 )

    ax2.set_ylabel('Cum KB Sent [' + msg_channel +']');  ax2.grid(True)
    ax2.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax2.set_ylim( bottom=0)
    ax2.set_title('TO THE ROBOT')

    ############################################################
    ax3.cla()
    #print "ax3:"
    #print got_rate
    if (got_rate):
      for i in range(0,len(sent_channels)-1): 
        if(i==0):
          ax3.fill_between( ratesent_KB.times[1:,0][:], 0 , ratesent_KB.v[1:,i][:],color=cols[i] , label=sent_channels[i]) 
        else:
          #print i
          #print cols[i]
          #print sent_channels
          #print ratesent_KB.times
          #print ratesent_KB.v
          ax3.fill_between( ratesent_KB.times[1:,0][:], ratesent_KB.v[1:,i-1][:] , ratesent_KB.v[1:,i][:],color=cols[i] ,
 label=sent_channels[i]) 
        ax3.plot(ratesent_KB.times[1:], np.transpose(ratesent_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=sent_channels[i])
      
      one_strip = np.ones( ( len(ratesent_KB.times[1:,0][:]),1) )
      ax3.plot(ratesent_KB.times[1:],  np.multiply(one_strip  ,4.01),'r:',linewidth=2 )
      ax3.plot(ratesent_KB.times[1:],  np.multiply(one_strip  ,1),'y:',linewidth=2 )
      ax3.plot(ratesent_KB.times[1:],  np.multiply(one_strip  ,0.25),'g:',linewidth=2 )

      ax3.set_ylabel('Rate KB Sent [' + msg_channel +']');  ax5.grid(True)
      ax3.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
      ax3.set_ylim( bottom=0)
    
    ############################################################
    ax4.cla()
    for i in range(0,len(received_channels)-1): 
      if(i==0):
        ax4.fill_between( cumreceived_KB.times[1:,0][:], 0 , cumreceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
      else:
        ax4.fill_between( cumreceived_KB.times[1:,0][:], cumreceived_KB.v[1:,i-1][:] , cumreceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
      ax4.plot(cumreceived_KB.times[1:], np.transpose(cumreceived_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=received_channels[i])
    ax4.set_ylabel('Cum KB Received [' + msg_channel +']');  ax4.grid(True)
    ax4.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax4.set_ylim( bottom=0)
    ax4.legend(loc=2,prop={'size':10})

    ############################################################
    ax5.cla()
    for i in range(0,len(received_channels)-1): 
      if(i==0):
        ax5.fill_between( cumreceived_KB.times[1:,0][:], 0 , cumreceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
      else:
        ax5.fill_between( cumreceived_KB.times[1:,0][:], cumreceived_KB.v[1:,i-1][:] , cumreceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
      ax5.plot(cumreceived_KB.times[1:], np.transpose(cumreceived_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=received_channels[i])

    one_strip = np.ones( ( len(cumreceived_KB.times[1:,0][:]),1) )
    ax5.plot(cumreceived_KB.times[1:,0][:],  np.multiply( cumreceived_KB.times[1:,0][:] , 256 ),'r:',linewidth=2 )
    ax5.plot(cumreceived_KB.times[1:,0][:],  np.multiply( cumreceived_KB.times[1:,0][:] , 128 ),'y:',linewidth=2 )
    ax5.plot(cumreceived_KB.times[1:,0][:],  np.multiply( cumreceived_KB.times[1:,0][:] , 64 ),'g:',linewidth=2 )

    ax5.set_ylabel('Cum KB Received [' + msg_channel +']');  ax5.grid(True)
    ax5.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
    ax5.set_ylim( bottom=0)
    ax5.set_title('FROM THE ROBOT')

    ############################################################
    ax6.cla()
    if (got_rate):
      for i in range(0,len(received_channels)-1): 
        if(i==0):
          ax6.fill_between( ratereceived_KB.times[1:,0][:], 0 , ratereceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
        else:
          ax6.fill_between( ratereceived_KB.times[1:,0][:], ratereceived_KB.v[1:,i-1][:] , ratereceived_KB.v[1:,i][:],color=cols[i] , label=received_channels[i]) 
      ax6.plot(ratereceived_KB.times[1:], np.transpose(ratereceived_KB.v[1:,i]),color=cols[i] ,  linewidth=1,label=received_channels[i])

      one_strip = np.ones( ( len(ratereceived_KB.times[1:,0][:]),1) )
      ax6.plot(ratereceived_KB.times[1:], np.multiply(one_strip  ,256.1),'r:',linewidth=2)
      ax6.plot(ratereceived_KB.times[1:], np.multiply(one_strip  ,128),'y:',linewidth=2 )
      ax6.plot(ratereceived_KB.times[1:], np.multiply(one_strip  ,64),'g:',linewidth=2 )

      ax6.set_ylabel('Rate KB Received [' + msg_channel +']');  ax6.grid(True)
      ax6.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime + front_block - first_utime)/1000000 )
      ax6.set_ylim( bottom=0)

  plt.plot()
  plt.draw()
  

# Microstrain INS/IMU Sensor:
def on_bw(channel, data):
  m = bandwidth_stats_t.decode(data)

  global rate_window, msg_last_rate, last_rate_utime, got_rate, msg_channel, sent_channels, received_channels
  #print len(sent_channels)
  #print len(m.sent_channels)
  if ( len(sent_channels) != len(m.sent_channels) ):
    print "number of messages has changed resetting [main]"
    got_rate=False
    reset_all()
    msg_channel = channel
    sent_channels = m.sent_channels
    received_channels = m.received_channels
    return

  msg_channel = channel
  sent_channels = m.sent_channels
  received_channels = m.received_channels

  which_utime = m.sim_utime # m.utime
  msgs.append(which_utime,m.queued_msgs)
  
  this_queued_KB= tuple([x/1024.0 for x in m.queued_bytes])
  queued_KB.append(which_utime,this_queued_KB)
  cumqueued_KB.append(which_utime,np.cumsum(this_queued_KB))

  this_sent_KB=tuple([x/1024.0 for x in m.sent_bytes]) 
  #sent_KB.append(which_utime,this_sent_KB)
  cumsent_KB.append(which_utime,np.cumsum(this_sent_KB))

  this_received_KB=tuple([x/1024.0 for x in m.received_bytes]) 
  #received_KB.append(which_utime,this_received_KB)
  cumreceived_KB.append(which_utime,np.cumsum(this_received_KB))
  #print len(this_received_KB)
  
  if (which_utime > last_rate_utime + rate_window):
    if (msg_last_rate.utime==0):
      print "dont rate"
      last_rate_utime = which_utime
      msg_last_rate = m
    else:
      elapsed_time = float((m.sim_utime - msg_last_rate.sim_utime)/1E6)
      #print "do rate"
      #print m.sim_utime
      #print msg_last_rate.sim_utime
      #print elapsed_time
      #if (msg_last_rate.num_sent_channels != m.num_sent_channels):
      #   print "number of messages has changed resetting"
      #   got_rate=False
      #   reset_all()
      #   last_rate_utime = which_utime
      #   msg_last_rate = m
      #   return

      last_sent_KB=tuple([x/1024.0 for x in msg_last_rate.sent_bytes]) 
      this_ratesent_KB = np.divide( np.subtract(np.array(this_sent_KB),np.array(last_sent_KB)) ,elapsed_time)
      ratesent_KB.append(which_utime,np.cumsum(this_ratesent_KB))
      #print "this ratesent:"
      #print this_ratesent_KB

      last_received_KB=tuple([x/1024.0 for x in msg_last_rate.received_bytes]) 
      this_ratereceived_KB = np.divide( np.subtract(np.array(this_received_KB),np.array(last_received_KB)) ,elapsed_time)
      ratereceived_KB.append(which_utime,np.cumsum(this_ratereceived_KB))
      #print this_ratesent_KB
      #print this_ratereceived_KB
      #
      print "got rate"
      got_rate=True
      last_rate_utime = which_utime
      msg_last_rate = m
      

  global last_utime
  if (which_utime < last_utime):
    print "out of order data, resetting now %s | last %s"   %(which_utime,last_utime)
    reset_all()
  last_utime = which_utime

  
#################################################################################
sent_channels=[]
received_channels=[]
msg_channel=''

lc = lcm.LCM()
print "started"
last_utime=0
first_utime=0
plot_window=30*1000000

rate_window= 1*1000000
last_rate_utime=0
msg_last_rate = bandwidth_stats_t()
msg_last_rate.utime =0
got_rate = False

msgs = SensorData(17); queued_KB = SensorData(17);
cumqueued_KB = SensorData(17); cumsent_KB = SensorData(17); cumreceived_KB = SensorData(17);

ratesent_KB = SensorData(17); ratereceived_KB = SensorData(17);

#left, bottom, width, height =0.07, 0.07, 0.395, 0.395
#box_ul = [left, 2*bottom+height, width, height]
#box_ur = [2*left+width, 2*bottom+height, width, height]
#box_ll = [left, bottom, width, height]
#box_lr = [2*left+width, bottom, width, height]
#fig1 = plt.figure(num=1, figsize=(14, 10), dpi=80, facecolor='w', edgecolor='k')
#ax1 = fig1.add_axes(box_ul)
#ax2 = fig1.add_axes(box_ur)
#ax3 = fig1.add_axes(box_ll)
#ax4 = fig1.add_axes(box_lr)

left, bottom, width, height =0.07, 0.07, 0.255, 0.395
box_1 = [left          , 2*bottom+height, width, height]
box_2 = [2*left+width  , 2*bottom+height, width, height]
box_3 = [3*left+2*width, 2*bottom+height, width, height]
box_4 = [left          , bottom, width, height]
box_5 = [2*left+width  , bottom, width, height]
box_6 = [3*left+2*width, bottom, width, height]


fig1 = plt.figure(num=1, figsize=(18, 12), dpi=80, facecolor='w', edgecolor='k')
ax1 = fig1.add_axes(box_1)
ax2 = fig1.add_axes(box_2)
ax3 = fig1.add_axes(box_3)
ax4 = fig1.add_axes(box_4)
ax5 = fig1.add_axes(box_5)
ax6 = fig1.add_axes(box_6)



plt.interactive(True)
plt.plot()
plt.draw()

def lcm_thread():
  sub1 = lc.subscribe("BASE_BW_STATS", on_bw) # required
  sub1 = lc.subscribe("ROBOT_BW_STATS", on_bw) # required
  
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
  if (dt_sec>1):
    print "drawing time: %f" %(dt_sec)
