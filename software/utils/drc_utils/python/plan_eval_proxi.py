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
from drc.behavior_t import behavior_t

def timestamp_now (): return int (time.time () * 1000000)

class State:
    def __init__(self):
        self.last_utime = 0
        self.init_status()
        self.manip_until_utime = 0
        self.walk_until_utime = 0
        self.seconds_per_step = 6 # conservative time per step
        self.current_behavior = 3
        self.last_utime_printf = 0

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
    # This offset is paired with planDesiredOffset in lcm2ros_ihmc.cpp
    planDesiredOffset = 1E6
    m = robot_plan_t.decode(data)
    print "got manip plan of length ", m.plan[m.num_states-1].utime*1E-6
    print "adding planDesiredOffset of ",planDesiredOffset*1E-6 , " seconds here"
    s.status.last_plan_msg_utime = s.last_utime
    s.status.last_plan_start_utime = s.last_utime
    s.manip_until_utime = s.last_utime + m.plan[m.num_states-1].utime + planDesiredOffset


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
        if (m.utime - s.last_utime_printf > 0.1*1E6):
            print "manip, time remaining: ", time_remaining
            s.last_utime_printf = m.utime

    elif (s.walk_until_utime > m.utime):
        # manip plan still being executed
        s.status.execution_status = 0 # EXECUTION_STATUS_EXECUTING
        s.status.plan_type = 2 # walking
        time_remaining = (s.walk_until_utime - m.utime)*1E-6
        if (m.utime - s.last_utime_printf > 0.1*1E6):
            print "walking, time remaining: ", time_remaining
            s.last_utime_printf = m.utime
    else:
        # manip or walking plan not being executed
        s.status.plan_type = 1 # standing
        s.status.execution_status = 2 # NO PLAN

    lc.publish("PLAN_EXECUTION_STATUS", s.status.encode())


def on_robot_behavior(channel, data):
    m = behavior_t.decode(data)
    if (s.current_behavior != m.behavior):
        if (m.behavior == 3):
            print "changed state from: ", s.current_behavior , " to " , m.behavior
            print "setting end of walking to be in 3 seconds"
            s.walk_until_utime = s.last_utime + 3*1E6
    s.current_behavior = m.behavior


lc = lcm.LCM()
print "started"
s = State();

sub1 = lc.subscribe("COMMITTED_ROBOT_PLAN", on_manip_plan)
sub2 = lc.subscribe("EST_ROBOT_STATE", on_est_robot_state)
sub3 = lc.subscribe("WALKING_CONTROLLER_PLAN_REQUEST", on_walking_plan_request)
sub4 = lc.subscribe("ROBOT_BEHAVIOR", on_robot_behavior)

while True:
    lc.handle()
