from py_drake_utils import quat2rpy
from collections import namedtuple
import pylab as pl

FootGoal = namedtuple('FootGoal', 'pos step_speed step_height step_id pos_fixed is_right_foot is_in_contact')

def decode_footstep_goal(goal_msg):
    rpy = quat2rpy([goal_msg.pos.rotation.w,
                    goal_msg.pos.rotation.x,
                    goal_msg.pos.rotation.y,
                    goal_msg.pos.rotation.z])
    goal = FootGoal(pos=pl.vstack([goal_msg.pos.translation.x,
                                   goal_msg.pos.translation.y,
                                   goal_msg.pos.translation.z,
                                   rpy]),
                    step_speed=goal_msg.step_speed,
                    step_height=goal_msg.step_height,
                    step_id=goal_msg.id,
                    pos_fixed=[goal_msg.fixed_x,
                               goal_msg.fixed_y,
                               goal_msg.fixed_z,
                               goal_msg.fixed_roll,
                               goal_msg.fixed_pitch,
                               goal_msg.fixed_yaw],
                    is_right_foot=goal_msg.is_right_foot,
                    is_in_contact=goal_msg.is_in_contact)
    if any(pl.isnan(goal.pos[[0,1,5]])):
        raise ValueError("I don't know how to handle NaN in x, y, or yaw")
    else:
        goal.pos[pl.find(pl.isnan(goal.pos))] = 0
    return goal

def decode_footstep_plan(plan_msg):
    footsteps = []
    for goal in plan_msg.footstep_goals:
        footsteps.append(decode_footstep_goal(goal))
    options = {'ignore_terrain': plan_msg.footstep_opts.ignore_terrain,
               'mu': plan_msg.footstep_opts.mu,
               'behavior': plan_msg.footstep_opts.behavior}
    return footsteps, options
