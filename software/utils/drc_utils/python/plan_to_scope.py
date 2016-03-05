#!/usr/bin/python
import os,sys
import lcm
import time

home_dir =os.getenv("HOME")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/site-packages")
sys.path.append(home_dir + "/drc/software/build/lib/python2.7/dist-packages")

from drc.robot_plan_t import robot_plan_t
from drc.plan_status_t import plan_status_t
from bot_core.robot_state_t import robot_state_t
from ihmc.last_received_message_t import last_received_message_t

def timestamp_now (): return int (time.time () * 1000000)

class State:
    def __init__(self):
        self.last_utime = 0
        self.last_received_utime = 0
        self.last_committed_plan = None

    def send_plan_with_offset(self, offset_utime):
        m = self.last_committed_plan
        for i in range(0,m.num_states):
            point = m.plan[i]
            point.utime = offset_utime + point.utime
            lc.publish("COMMITTED_ROBOT_PLAN_STATES", point.encode())


def on_manip_plan(channel, data):
    m = robot_plan_t.decode(data)
    print "got plan msg at" , s.last_utime , "with" , m.num_states , "states"
    #print m.plan[m.num_states-1].utime
    s.last_committed_plan = m
    #s.send_plan_with_offset(s.last_utime)



def on_est_robot_state(channel, data):
    m = robot_state_t.decode(data)
    s.last_utime = m.utime

def on_last_received(channel, data):
    m = last_received_message_t.decode(data)
    if (m.receive_timestamp != s.last_received_utime):
        s.last_received_utime = m.receive_timestamp
        print "got imhc ack at" , s.last_received_utime
        if (s.last_committed_plan is not None):
            # resend the plan as samples offsetting by the time that ihmc received it
            s.send_plan_with_offset(s.last_received_utime)


lc = lcm.LCM()
print "started"
s = State();

sub1 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_manip_plan)
sub2 = lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
sub3 = lc.subscribe("IHMC_LAST_RECEIVED", on_last_received)

while True:
    lc.handle()
