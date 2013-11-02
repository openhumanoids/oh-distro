import drc
import pylab as pl
import numpy as np
from collections import namedtuple

import py_drake_utils as ut
from bdi_step.utils import Behavior
from py_drake_utils import quat2rpy

# Experimentally determined vector relating BDI's frame for foot position to ours. This is the xyz vector from the position of the foot origin (from drake forwardKin) to the BDI Atlas foot pos estimate, expressed in the frame of the foot. It is the same to within 10^-4 for both feet.
ATLAS_FRAME_OFFSET = np.matrix([[0.0600], [0.000], [-0.0850]])

BaseFootGoal = namedtuple('FootGoal', 'pos step_speed step_height step_id pos_fixed is_right_foot is_in_contact bdi_step_duration bdi_sway_duration bdi_lift_height bdi_toe_off bdi_knee_nominal')

class FootGoal(BaseFootGoal):
    def to_bdi_spec(self, behavior, step_index):
        if behavior == Behavior.BDI_STEPPING:
            return self.to_step_spec(step_index)
        elif behavior == Behavior.BDI_WALKING:
            return self.to_walk_spec(step_index)

    def to_step_spec(self, step_index):
        step_spec = drc.atlas_behavior_step_spec_t()
        step_spec.step_index = step_index
        step_spec.foot_index = self.is_right_foot
        step_spec.foot = self.to_bdi_foot_data()
        step_spec.action = self.to_step_action()
        return step_spec

    def to_walk_spec(self, step_index):
        walk_spec = drc.atlas_behavior_walk_spec_t()
        walk_spec.step_index = step_index
        walk_spec.foot_index = self.is_right_foot
        walk_spec.foot = self.to_bdi_foot_data()
        walk_spec.action = self.to_walk_action()
        return walk_spec

    def to_bdi_foot_data(self):
        foot_data = drc.atlas_behavior_foot_data_t()
        foot_data.position = self.to_atlas_frame(self.pos)[:3]
        foot_data.yaw = self.pos[5]
        foot_data.normal = ut.rpy2rotmat(self.pos[3:6,0]) * np.matrix([[0],[0],[1]])
        return foot_data

    def to_step_action(self):
        action = drc.atlas_behavior_step_action_t()
        action.swing_height = self.step_height
        action.step_duration = self.bdi_step_duration
        action.sway_duration = self.bdi_sway_duration
        action.lift_height = self.bdi_lift_height
        action.toe_off = self.bdi_toe_off
        action.knee_nominal = self.bdi_knee_nominal
        return action

    def to_walk_action(self):
        action = drc.atlas_behavior_walk_action_t()
        action.step_duration = self.bdi_step_duration
        action.swing_height = self.step_height
        return action

    def to_step_data(self, step_index):
        """
        Deprecated in favor of to_bdi_spec, and no longer tested
        """
        step_data = drc.atlas_step_data_t()
        step_data.step_index = step_index
        step_data.foot_index = self.is_right_foot
        step_data.duration = self.bdi_step_duration
        # print "Footstep pos: ", self.pos
        pos = self.to_atlas_frame(self.pos)
        step_data.position = pos[:3]
        # print "Transformed pos: ", step_data.position
        step_data.yaw = self.pos[5]
        step_data.normal = ut.rpy2rotmat(self.pos[3:6,0]) * np.matrix([[0],[0],[1]])
        step_data.swing_height = self.step_height
        return step_data

    @staticmethod
    def to_atlas_frame(footpos):
        """
        Convert a foot position from our representation (foot orig) to what BDI's walker expects (the position of a point near the center of the sole)
        """
        R = ut.rpy2rotmat(footpos[3:,0])
        offs = R * ATLAS_FRAME_OFFSET
        footpos[:3] += offs
        return footpos

    @staticmethod
    def from_goal_msg(goal_msg):
        rpy = quat2rpy([goal_msg.pos.rotation.w,
                        goal_msg.pos.rotation.x,
                        goal_msg.pos.rotation.y,
                        goal_msg.pos.rotation.z])
        if goal_msg.bdi_lift_height == 0:
            # A lift height of 0 is ignored by the API, so we'll replace it with a very small but nonzero value (1cm).
            goal_msg.bdi_lift_height = 0.01
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
                        is_in_contact=goal_msg.is_in_contact,
                        bdi_step_duration=goal_msg.bdi_step_duration,
                        bdi_sway_duration=goal_msg.bdi_sway_duration,
                        bdi_lift_height=goal_msg.bdi_lift_height,
                        bdi_toe_off=goal_msg.bdi_toe_off,
                        bdi_knee_nominal=goal_msg.bdi_knee_nominal)
        if any(pl.isnan(goal.pos[[0,1,5]])):
            raise ValueError("I don't know how to handle NaN in x, y, or yaw")
        else:
            goal.pos[pl.find(pl.isnan(goal.pos))] = 0
        return goal

def decode_footstep_plan(plan_msg):
    footsteps = []
    for goal in plan_msg.footstep_goals:
        footsteps.append(FootGoal.from_goal_msg(goal))
    options = {'ignore_terrain': plan_msg.footstep_opts.ignore_terrain,
               'mu': plan_msg.footstep_opts.mu,
               'behavior': plan_msg.footstep_opts.behavior}
    return footsteps, options

