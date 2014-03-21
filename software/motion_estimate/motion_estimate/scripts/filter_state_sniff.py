#!/usr/bin/python

import os,sys
import lcm
import time
from lcm import LCM
import math
import numpy  as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab

sys.path.append("/home/mfallon/other_projects/Fixie/build/lib/python2.7/site-packages")
sys.path.append("/home/mfallon/other_projects/Fixie/build/lib/python2.7/dist-packages")
from bot_core.pose_t import pose_t
from mav.filter_state_t import filter_state_t
from microstrain.ins_t import ins_t
from fovis.update_t import update_t
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
  pos.reset(); posrate.reset()
  ang.reset(); angrate.reset(); 
  ins_angrate.reset(); 
  vo_posdelta.reset(); vo_posrate.reset(); 
  vo_angdelta.reset(); vo_angrate.reset();


def quat_to_euler(q) :
  roll = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]));
  pitch = math.asin(2*(q[0]*q[2]-q[3]*q[1]));
  yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
  return [roll,pitch,yaw]

def plot_data():
  global last_plot_time, last_utime
  nowtime = time.time ()
  if (nowtime < last_plot_time +plot_timing):
    #print "wait"
    return
  last_plot_time = nowtime
  if ( len(pos.utimes) >1):
    plt.figure(1)
    ############################################################
    ax1.cla()
    ax1.plot(pos.utimes[1:], np.transpose(pos.v[1:,0]), 'r', linewidth=1,label="x")
    ax1.plot(pos.utimes[1:], np.transpose(pos.v[1:,1]), 'g', linewidth=1,label="y")
    ax1.plot(pos.utimes[1:], np.transpose(pos.v[1:,2]), 'b', linewidth=1,label="z")
    ax1.legend();  ax1.set_xlabel('Time '+ str(last_utime));  ax1.set_ylabel('Local Pos [m]');  ax1.grid(True)
    ax1.legend(loc=2,prop={'size':10})
    ax1.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    ############################################################
    ax2.cla()
    ax2.plot(posrate.utimes[1:], np.transpose(posrate.v[1:,0]), 'r', linewidth=1,label="u fwd")
    ax2.plot(posrate.utimes[1:], np.transpose(posrate.v[1:,1]), 'g', linewidth=1,label="v left")
    ax2.plot(posrate.utimes[1:], np.transpose(posrate.v[1:,2]), 'b', linewidth=1,label="w up")
    # also add VO posrates: [linear velocity]
    ax2.plot(vo_posrate.utimes[1:],  np.transpose(vo_posrate.v[1:,0]), 'r+', linewidth=1,label="VO u fwd")
    ax2.plot(vo_posrate.utimes[1:],  np.transpose(vo_posrate.v[1:,1]), 'g+', linewidth=1,label="VO v left")
    ax2.plot(vo_posrate.utimes[1:],  np.transpose(vo_posrate.v[1:,2]), 'b+', linewidth=1,label="VO w up")
    ax2.legend();   ax2.set_xlabel('Time - vo: ' + str(len(vo_posrate.utimes)));   ax2.set_ylabel('Body Pos Rate [m/s]');   ax2.grid(True)
    ax2.legend(loc=2,prop={'size':10})
    ax2.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    #ax2.set_ylim(-2,2)
    ############################################################
    ax3.cla()
    ax3.plot(ang.utimes[1:], np.transpose(ang.v[1:,0]) *180/math.pi , 'r', linewidth=1,label="roll")
    ax3.plot(ang.utimes[1:], np.transpose(ang.v[1:,1]) *180/math.pi , 'm', linewidth=1,label="pitch")
    ax3.plot(ang.utimes[1:], np.transpose(ang.v[1:,2]) *180/math.pi , 'y', linewidth=1,label="yaw")
    ax3.legend();   ax3.set_xlabel('Time - state: ' + str(len(ang.utimes)));   ax3.set_ylabel('Local Angle [Deg]');   ax3.grid(True)
    ax3.legend(loc=2,prop={'size':10})
    ax3.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    ############################################################
    ax4.cla()
    ax4.plot(angrate.utimes[1:], np.transpose(angrate.v[1:,0]), 'r', linewidth=1,label="p")
    ax4.plot(angrate.utimes[1:], np.transpose(angrate.v[1:,1]), 'm', linewidth=1,label="q")
    ax4.plot(angrate.utimes[1:], np.transpose(angrate.v[1:,2]), 'y', linewidth=1,label="r")
    ax4.plot(ins_angrate.utimes[1:], -1* np.transpose(ins_angrate.v[1:,0]), 'r:', linewidth=1,label="-g0")
    ax4.plot(ins_angrate.utimes[1:],     np.transpose(ins_angrate.v[1:,1]), 'm:', linewidth=1,label="g1")
    ax4.plot(ins_angrate.utimes[1:], -1* np.transpose(ins_angrate.v[1:,2]), 'y:', linewidth=1,label="-g2")
    # also add VO angrates:
    ax4.plot(vo_angrate.utimes[1:],  np.transpose(vo_angrate.v[1:,0]), 'r+', linewidth=1,label="VO roll rate")
    ax4.plot(vo_angrate.utimes[1:],  np.transpose(vo_angrate.v[1:,1]), 'm+', linewidth=1,label="VO pitch rate")
    ax4.plot(vo_angrate.utimes[1:],  np.transpose(vo_angrate.v[1:,2]), 'y+', linewidth=1,label="VO yaw rate")
    ax4.legend();   ax4.set_xlabel('Time - ins: ' + str(len(ins_angrate.utimes)));  ax4.set_ylabel('Body Angle Rate [Deg/s]');   ax4.grid(True)
    ax4.legend(loc=2,prop={'size':10})
    ax4.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )


  if ( len(vo_angdelta.utimes) >1):
    # VO time (skip 2nd one also as its always zero:)
    plt.figure(2)
    ax5.cla()
    ax5.plot(vo_angdelta.utimes[2:],     np.transpose(vo_angdelta.v[2:,3]), 'm+', linewidth=1,label="VO delta time")
    ax5.legend(loc=2,prop={'size':10});   ax5.set_xlabel('Time '+ str(last_utime));   ax5.set_ylabel('Delta Timestamp [s]');   ax5.grid(True)
    ax5.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    # Vo angle rates:
    ax6.cla()
    ax6.plot(vo_angrate.utimes[1:],     np.transpose(vo_angrate.v[1:,0]), 'm+', linewidth=1,label="pitch rate")
    ax6.plot(vo_angrate.utimes[1:],     np.transpose(vo_angrate.v[1:,1]), 'y+', linewidth=1,label="yaw rate")
    ax6.plot(vo_angrate.utimes[1:],     np.transpose(vo_angrate.v[1:,2]), 'r+', linewidth=1,label="roll rate")
    ax6.legend();   ax6.set_xlabel('Time');   ax6.set_ylabel('VO Body Angle Rate [Deg/s]');   ax6.grid(True)
    ax6.legend(loc=2,prop={'size':10})
    ax6.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    # VO angle delta - measured change in angle:
    ax7.cla()
    ax7.plot(vo_angdelta.utimes[1:],  np.transpose(vo_angdelta.v[1:,0]), 'r+', linewidth=1,label="roll")
    ax7.plot(vo_angdelta.utimes[1:],  np.transpose(vo_angdelta.v[1:,1]), 'm+', linewidth=1,label="pitch")
    ax7.plot(vo_angdelta.utimes[1:],  np.transpose(vo_angdelta.v[1:,2]), 'y+', linewidth=1,label="yaw")
    ax7.legend();   ax7.set_xlabel('Time');   ax7.set_ylabel('VO Body Angle Delta [Deg]');   ax7.grid(True)
    ax7.legend(loc=2,prop={'size':10})
    ax7.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
    # vo_posdelta - measured change in position:
    ax8.cla()
    ax8.plot(vo_posdelta.utimes[1:],  np.transpose(vo_posdelta.v[1:,0]), 'r+', linewidth=1,label="dx fwd")
    ax8.plot(vo_posdelta.utimes[1:],  np.transpose(vo_posdelta.v[1:,1]), 'g+', linewidth=1,label="dy left")
    ax8.plot(vo_posdelta.utimes[1:],  np.transpose(vo_posdelta.v[1:,2]), 'b+', linewidth=1,label="dz up")
    ax8.legend();   ax8.set_xlabel('Time');   ax8.set_ylabel('VO Body Delta Pos [m]');   ax8.grid(True)
    ax8.legend(loc=2,prop={'size':10})
    ax8.set_xlim( (last_utime - plot_window - first_utime)/1000000 , (last_utime - first_utime)/1000000 )
  plt.plot()
  plt.draw()


def on_relvo(channel, data):
  m = update_t.decode(data)
  #print "K T:      %.3f" % (m.timestamp)
  #print "KPT:      %.3f" % (m.prev_timestamp)
  pyr=quat_to_euler(m.rotation)
  vo_delta_time = (m.timestamp - m.prev_timestamp)/1000000.0
  #convert camera pyr to body rpy #########################
  #TODO: use rotation matrix
  rpyt = [pyr[2], -pyr[0], -pyr[1], vo_delta_time]
  vo_angdelta.append(m.timestamp,rpyt)
  rpy_rate_t = [pyr[2]/vo_delta_time, -pyr[0]/vo_delta_time, -pyr[1]/vo_delta_time, vo_delta_time]
  vo_angrate.append(m.timestamp,rpy_rate_t)
  #covert camera translation into body translation ########
  posdelta_t = [m.translation[2], -m.translation[0], -m.translation[1], vo_delta_time]
  vo_posdelta.append(m.timestamp,posdelta_t)
  posrate_t = [m.translation[2]/vo_delta_time, -m.translation[0]/vo_delta_time, -m.translation[1]/vo_delta_time, vo_delta_time]
  vo_posrate.append(m.timestamp,posrate_t)
  #global last_utime
  #if (m.timestamp < last_utime):
  #  print "out of order data, resetting now %s | last %s"   %(m.timestamp,last_utime)
  #  reset_all()
  #last_utime = m.timestamp
  plot_data()

def on_ins(channel, data):
  m = ins_t.decode(data)
  ins_angrate.append(m.utime,m.gyro)
  #print "IMU:      %.3f" % (m.utime)
  #plot_data()

def on_pose(channel, data):
  m = pose_t.decode(data)
  #print "p %s" % (p.utime)
  dt = float(  timestamp_now() - m.utime )/1000000
  #print "POSE:      %.3f" % (dt)

def on_state(channel, data):
  m = filter_state_t.decode(data)
  global first_utime
  if (first_utime==0):
    first_utime= m.utime
  angrate.append(m.utime,  [m.state[0], m.state[1], m.state[2]] )
  posrate.append(m.utime,  [m.state[3], m.state[4], m.state[5]] )
  pos.append(m.utime,  [m.state[9], m.state[10], m.state[11]] )
  ang.append(m.utime,quat_to_euler(m.quat))
  global last_utime
  if (m.utime < last_utime):
    print "out of order data, resetting now %s | last %s"   %(m.utime,last_utime)
    reset_all()
  last_utime = m.utime
  plot_data()
  #print "STATE     %.3f" % (m.utime)




#################################################################################
lc = lcm.LCM()
print "started"
last_plot_time = 0
last_utime=0
first_utime=0
plot_window=3*1000000 #3sec
plot_timing=3 # 3 second between plot updates
pos = SensorData(3); posrate = SensorData(3); 
ang = SensorData(3); angrate = SensorData(3); 
ins_angrate = SensorData(3); 
vo_posdelta = SensorData(4); vo_posrate = SensorData(4); 
vo_angdelta = SensorData(4); vo_angrate = SensorData(4);



left, bottom, width, height =0.07, 0.07, 0.395, 0.395
box_ul = [left, 2*bottom+height, width, height]
box_ur = [2*left+width, 2*bottom+height, width, height]
box_ll = [left, bottom, width, height]
box_lr = [2*left+width, bottom, width, height]

fig2 = plt.figure(num=2, figsize=(14, 10), dpi=80, facecolor='w', edgecolor='k')
ax5 = fig2.add_axes(box_ul)
ax6 = fig2.add_axes(box_ur)
ax7 = fig2.add_axes(box_ll)
ax8 = fig2.add_axes(box_lr)

plt.interactive(True)
plt.plot()
plt.draw()

fig1 = plt.figure(num=1, figsize=(14, 10), dpi=80, facecolor='w', edgecolor='k')
ax1 = fig1.add_axes(box_ul)
ax2 = fig1.add_axes(box_ur)
ax3 = fig1.add_axes(box_ll)
ax4 = fig1.add_axes(box_lr)


sub1 = lc.subscribe("STATE_ESTIMATOR_POSE", on_pose)
sub2 = lc.subscribe("STATE_ESTIMATOR_STATE", on_state)
sub3 = lc.subscribe("MICROSTRAIN_INS", on_ins)
sub4 = lc.subscribe("KINECT_REL_ODOMETRY", on_relvo)

while True:
  ## Handle LCM if new messages have arrived.
  lc.handle()

lc.unsubscribe(sub1)
lc.unsubscribe(sub2)
lc.unsubscribe(sub3)
lc.unsubscribe(sub4)
