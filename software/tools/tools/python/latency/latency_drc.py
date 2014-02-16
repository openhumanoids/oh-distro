#!/usr/bin/python
# Filename: latency_module.py
# Simple module for calculating delay
# M Fallon
import time


class Latency:
  def __init__(self, counter, prev_utime):
    self.js_time = []
    self.js_walltime = []
    self.latency_cumsum =0
    self.latency_step_cumsum =0
    self.latency_count =0
    self.p = 200
    self.pf = 200.0
  def timestamp_now (self): return int (time.time () * 1000000)
  def timestamp_now_sec (self): return time.time () 
  def add_from(self, js_time, js_walltime):
    self.js_time.append(js_time)
    self.js_walltime.append(js_walltime)
  def add_to(self, jc_time, jc_walltime,message):
    if (jc_time in self.js_time):
      idx=self.js_time.index(jc_time)
      step_latency = len( self.js_time ) - 1 - idx
      elapsed_walltime = jc_walltime - self.js_walltime[idx]
      if (1==0):
        #print s.js_time
        print "current: %d" % jc_time
        print idx
        print step_latency
        print self.js_walltime[idx]
        print jc_walltime
        print elapsed_walltime
      #print "%d | %d newer msgs | %f sec delay [last]" % (jc_time, step_latency, elapsed_walltime )
      self.js_walltime =[]
      self.js_time=[]
      self.latency_cumsum += elapsed_walltime
      self.latency_count += 1
      self.latency_step_cumsum += step_latency
      if not self.latency_count % self.p:
        print "%s | %d | %f newer msgs | %f sec delay [%d]" % (message,jc_time, (self.latency_step_cumsum/self.pf), self.latency_cumsum/self.pf, self.latency_count )
        self.latency_count =0
        self.latency_cumsum=0 
        self.latency_step_cumsum=0 
