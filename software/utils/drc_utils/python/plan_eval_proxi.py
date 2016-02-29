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
from drc.walking_plan_request_t import walking_plan_request_t

def timestamp_now (): return int (time.time () * 1000000)

class State:
    def __init__(self):
        self.last_utime = 0
        self.init_status()
        self.manip_until_utime = 0
        self.walk_until_utime = 0
        self.seconds_per_step = 2.9 # guesstimated time per step

    def init_status(self):
        self.status = plan_status_t()
        self.status.utime = 0
        self.status.execution_status = 0
        self.status.last_plan_msg_utime = 0
        self.status.last_plan_start_utime = 0
        self.status.plan_type = 0
        self.status.recovery_enabled = False
        self.status.bracing_enabled = False


def on_manip_plan(channel, data):
    m = robot_plan_t.decode(data)
    print "got manip plan"
    print m.plan[m.num_states-1].utime
    s.status.last_plan_msg_utime = s.last_utime
    s.status.last_plan_start_utime = s.last_utime
    s.manip_until_utime = s.last_utime + m.plan[m.num_states-1].utime


def on_walking_plan_request(channel, data):
    m = walking_plan_request_t.decode(data)
    print "got walking plan request: ",  m.footstep_plan.num_steps , " steps - 2 initial and ", (m.footstep_plan.num_steps-2), " actual"
    s.status.last_plan_msg_utime = s.last_utime
    s.status.last_plan_start_utime = s.last_utime
    s.walk_until_utime = s.last_utime + (m.footstep_plan.num_steps-2)*s.seconds_per_step*1E6

def on_est_robot_state(channel, data):
    m = robot_state_t.decode(data)
    s.last_utime = m.utime
    s.status.utime = m.utime

    if (s.manip_until_utime > m.utime):
        # manip plan still being executed
        s.status.execution_status = 0 # EXECUTION_STATUS_EXECUTING
        s.status.plan_type = 8 # manip
        time_remaining = (s.manip_until_utime - m.utime)*1E-6
        print "manip, time remaining: ", time_remaining
    elif (s.walk_until_utime > m.utime):
        # manip plan still being executed
        s.status.execution_status = 0 # EXECUTION_STATUS_EXECUTING
        s.status.plan_type = 2 # walking
        time_remaining = (s.walk_until_utime - m.utime)*1E-6
        print "walking, time remaining: ", time_remaining
    else:
        # manip or walking plan not being executed
        s.status.plan_type = 1 # standing
        s.status.execution_status = 2 # NO PLAN

    lc.publish("PLAN_EXECUTION_STATUS", s.status.encode())


lc = lcm.LCM()
print "started"
s = State();

sub1 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_manip_plan)
sub2 = lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
sub3 = lc.subscribe("WALKING_CONTROLLER_PLAN_REQUEST", on_walking_plan_request)

while True:
    lc.handle()
