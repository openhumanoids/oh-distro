import drc
import atlas
import pylab as pl
import numpy as np
# from collections import namedtuple

import py_drake_utils as ut
from bdi_step.utils import Behavior
from py_drake_utils import quat2rpy

MAX_LIFT_HEIGHT = 0.40;


class FootGoal:
    def __init__(self,
                 pos,
                 step_speed=1.0,
                 step_height=0.05,
                 step_id=0,
                 pos_fixed=np.ones((6,1)),
                 is_right_foot=True,
                 is_in_contact=True,
                 bdi_step_duration=0.0,
                 bdi_sway_duration=0.0,
                 bdi_lift_height=0.0,
                 bdi_toe_off=atlas.behavior_step_action_t.TOE_OFF_ENABLE,
                 bdi_knee_nominal=0.0,
                 bdi_max_body_accel=0.0,
                 bdi_max_foot_vel=0.0,
                 bdi_sway_end_dist=0.02,
                 bdi_step_end_dist=0.02,
                 support_contact_groups=drc.footstep_params_t.SUPPORT_GROUPS_HEEL_TOE,
                 terrain_pts=np.zeros((2,1))):
        self.pos = pos
        self.step_speed = step_speed
        self.step_height = step_height
        self.step_id = step_id
        self.pos_fixed = pos_fixed
        self.is_right_foot = is_right_foot
        self.is_in_contact = is_in_contact
        self.bdi_step_duration = bdi_step_duration
        self.bdi_sway_duration = bdi_sway_duration
        self.bdi_lift_height = bdi_lift_height
        self.bdi_toe_off = bdi_toe_off
        self.bdi_knee_nominal = bdi_knee_nominal
        self.bdi_max_body_accel = bdi_max_body_accel
        self.bdi_max_foot_vel = bdi_max_foot_vel
        self.bdi_sway_end_dist = bdi_sway_end_dist
        self.bdi_step_end_dist = bdi_step_end_dist
        self.support_contact_groups = support_contact_groups
        self.terrain_pts = terrain_pts

    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return str(self)

    def to_bdi_spec(self, behavior, step_index):
        if behavior == Behavior.BDI_STEPPING:
            return self.to_step_spec(step_index)
        elif behavior == Behavior.BDI_WALKING:
            return self.to_walk_spec(step_index)

    def to_step_spec(self, step_index):
        step_spec = atlas.behavior_step_spec_t()
        step_spec.step_index = step_index
        step_spec.foot_index = self.is_right_foot
        step_spec.foot = self.to_bdi_foot_data()
        step_spec.action = self.to_step_action()
        return step_spec

    def to_walk_spec(self, step_index):
        walk_spec = atlas.behavior_walk_spec_t()
        walk_spec.step_index = step_index
        walk_spec.foot_index = self.is_right_foot
        walk_spec.foot = self.to_bdi_foot_data()
        walk_spec.action = self.to_walk_action()
        return walk_spec

    def to_bdi_foot_data(self):
        foot_data = atlas.behavior_foot_data_t()
        # foot_data.position = self.to_atlas_frame(self.pos)[:3]
        foot_data.position = self.pos[:3]
        foot_data.yaw = self.pos[5]
        foot_data.normal = ut.rpy2rotmat(self.pos[3:]).dot(np.array([0,0,1]))
        return foot_data

    def to_step_action(self):
        action = atlas.behavior_step_action_t()
        action.swing_height = self.step_height
        action.step_duration = self.bdi_step_duration
        action.sway_duration = self.bdi_sway_duration
        if self.terrain_pts.shape[1] >= 2:
            max_step_height = max(self.terrain_pts[1,0], self.terrain_pts[1,-1])
            action.lift_height = max(self.terrain_pts[1,:]) - max_step_height + self.bdi_lift_height
            action.lift_height = min(action.lift_height, MAX_LIFT_HEIGHT);
        else:
            action.lift_height = self.bdi_lift_height
        action.lift_height = max(action.lift_height, 0.01)  # A lift height of 0 is ignored by the API, so we'll replace it with a very small but nonzero value (1cm).
        action.toe_off = self.bdi_toe_off
        action.knee_nominal = self.bdi_knee_nominal
        action.max_body_accel = self.bdi_max_body_accel
        action.max_foot_vel = self.bdi_max_foot_vel
        action.sway_end_dist = self.bdi_sway_end_dist
        action.step_end_dist = self.bdi_step_end_dist
        return action

    def to_walk_action(self):
        action = atlas.behavior_walk_action_t()
        action.step_duration = self.bdi_step_duration
        action.swing_height = self.step_height
        return action

    def to_footstep_t(self):
        msg = drc.footstep_t()
        msg.pos = drc.position_3d_t();
        msg.pos.translation = drc.vector_3d_t();
        msg.pos.translation.x, msg.pos.translation.y, msg.pos.translation.z = self.pos[:3]
        msg.pos.rotation = drc.quaternion_t();
        msg.pos.rotation.w, msg.pos.rotation.x, msg.pos.rotation.y, msg.pos.rotation.z = ut.rpy2quat(self.pos[3:])
        msg.id = self.step_id
        msg.is_right_foot = self.is_right_foot
        msg.is_in_contact = self.is_in_contact
        msg.fixed_x, msg.fixed_y, msg.fixed_z, msg.fixed_roll, msg.fixed_pitch, msg.fixed_yaw = self.pos_fixed
        msg.num_terrain_pts = self.terrain_pts.shape[1]
        if msg.num_terrain_pts > 0:
            msg.terrain_path_dist = self.terrain_pts[0,:]
            msg.terrain_height = self.terrain_pts[1,:]
        msg.params = drc.footstep_params_t(); # TODO: this should probably be filled in
        msg.params.bdi_step_duration=self.bdi_step_duration
        msg.params.bdi_sway_duration=self.bdi_sway_duration
        msg.params.bdi_lift_height=self.bdi_lift_height
        msg.params.bdi_toe_off=self.bdi_toe_off
        msg.params.bdi_knee_nominal=self.bdi_knee_nominal
        msg.params.bdi_max_body_accel=self.bdi_max_body_accel
        msg.params.bdi_max_foot_vel=self.bdi_max_foot_vel
        msg.params.bdi_sway_end_dist=self.bdi_sway_end_dist
        msg.params.bdi_step_end_dist=self.bdi_step_end_dist
        msg.params.support_contact_groups=self.support_contact_groups
        return msg

    # @staticmethod
    # def to_atlas_frame(footpos):
    #     """
    #     Convert a foot position from our representation (foot orig) to what BDI's walker expects (the position of a point near the center of the sole)
    #     """
    #     R = ut.rpy2rotmat(footpos[3:,0])
    #     offs = R * ATLAS_FRAME_OFFSET
    #     footpos[:3] += offs
    #     return footpos

    @staticmethod
    def from_goal_msg(goal_msg):
        rpy = quat2rpy([goal_msg.pos.rotation.w,
                        goal_msg.pos.rotation.x,
                        goal_msg.pos.rotation.y,
                        goal_msg.pos.rotation.z])
        goal = FootGoal(pos=pl.hstack([goal_msg.pos.translation.x,
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
                        bdi_knee_nominal=goal_msg.bdi_knee_nominal,
                        bdi_max_body_accel=goal_msg.bdi_max_body_accel,
                        bdi_max_foot_vel=goal_msg.bdi_max_foot_vel,
                        bdi_sway_end_dist=goal_msg.bdi_sway_end_dist,
                        bdi_step_end_dist=goal_msg.bdi_step_end_dist,
                        support_contact_groups=goal_msg.support_contact_groups,
                        terrain_pts=pl.vstack([goal_msg.terrain_path_dist, goal_msg.terrain_height]))
        if any(pl.isnan(goal.pos[[0,1,5]])):
            raise ValueError("I don't know how to handle NaN in x, y, or yaw")
        else:
            goal.pos[pl.find(pl.isnan(goal.pos))] = 0
        return goal

    @staticmethod
    def from_footstep_msg(goal_msg):
        rpy = quat2rpy([goal_msg.pos.rotation.w,
                        goal_msg.pos.rotation.x,
                        goal_msg.pos.rotation.y,
                        goal_msg.pos.rotation.z])
        goal = FootGoal(pos=pl.hstack([goal_msg.pos.translation.x,
                                       goal_msg.pos.translation.y,
                                       goal_msg.pos.translation.z,
                                       rpy]),
                        step_speed=goal_msg.params.step_speed,
                        step_height=goal_msg.params.step_height,
                        step_id=goal_msg.id,
                        pos_fixed=[goal_msg.fixed_x,
                                   goal_msg.fixed_y,
                                   goal_msg.fixed_z,
                                   goal_msg.fixed_roll,
                                   goal_msg.fixed_pitch,
                                   goal_msg.fixed_yaw],
                        is_right_foot=goal_msg.is_right_foot,
                        is_in_contact=goal_msg.is_in_contact,
                        bdi_step_duration=goal_msg.params.bdi_step_duration,
                        bdi_sway_duration=goal_msg.params.bdi_sway_duration,
                        bdi_lift_height=goal_msg.params.bdi_lift_height,
                        bdi_toe_off=goal_msg.params.bdi_toe_off,
                        bdi_knee_nominal=goal_msg.params.bdi_knee_nominal,
                        bdi_max_body_accel=goal_msg.params.bdi_max_body_accel,
                        bdi_max_foot_vel=goal_msg.params.bdi_max_foot_vel,
                        bdi_sway_end_dist=goal_msg.params.bdi_sway_end_dist,
                        bdi_step_end_dist=goal_msg.params.bdi_step_end_dist,
                        support_contact_groups=goal_msg.params.support_contact_groups,
                        terrain_pts=pl.vstack([goal_msg.terrain_path_dist, goal_msg.terrain_height]))
        if any(pl.isnan(goal.pos[[0,1,5]])):
            raise ValueError("I don't know how to handle NaN in x, y, or yaw")
        else:
            goal.pos[pl.find(pl.isnan(goal.pos))] = 0
        return goal

    def copy(self):
        return FootGoal(pos = np.copy(self.pos),
                        step_speed = self.step_speed,
                        step_height = self.step_height,
                        step_id = self.step_id,
                        pos_fixed = np.copy(self.pos_fixed),
                        is_right_foot = self.is_right_foot,
                        is_in_contact = self.is_in_contact,
                        bdi_step_duration = self.bdi_step_duration,
                        bdi_sway_duration = self.bdi_sway_duration,
                        bdi_lift_height = self.bdi_lift_height,
                        bdi_toe_off = self.bdi_toe_off,
                        bdi_knee_nominal = self.bdi_knee_nominal,
                        bdi_max_body_accel = self.bdi_max_body_accel,
                        bdi_max_foot_vel = self.bdi_max_foot_vel,
                        bdi_sway_end_dist = self.bdi_sway_end_dist,
                        bdi_step_end_dist = self.bdi_step_end_dist,
                        support_contact_groups=self.support_contact_groups,
                        terrain_pts = np.copy(self.terrain_pts))

def decode_deprecated_footstep_plan(plan_msg):
    footsteps = []
    for goal in plan_msg.footstep_goals:
        footsteps.append(FootGoal.from_goal_msg(goal))
    options = {'ignore_terrain': plan_msg.footstep_opts.ignore_terrain,
               'mu': plan_msg.footstep_opts.mu,
               'behavior': plan_msg.footstep_opts.behavior}
    return footsteps, options

def decode_footstep_plan(plan_msg):
    footsteps = []
    for step in plan_msg.footsteps:
        footsteps.append(FootGoal.from_footstep_msg(step))
    options = {'ignore_terrain': False,
               'mu': 1.0,
               'behavior': plan_msg.params.behavior}
    return footsteps, options

def encode_footstep_plan(footsteps, params):
    plan = drc.footstep_plan_t()
    plan.num_steps = len(footsteps)
    plan.footsteps = [step.to_footstep_t() for step in footsteps]
    if params is not None:
        plan.params = params
    else:
        plan.params = drc.footstep_plan_params_t()
    plan.iris_region_assignments = [-1 for f in footsteps]
    return plan
