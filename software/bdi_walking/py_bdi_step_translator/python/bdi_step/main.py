from __future__ import division
import numpy as np
import lcm
import drc
import py_drake_utils as ut
import bdi_step.footsteps

NUM_REQUIRED_WALK_STEPS = 4


def translate_footsteps_to_bdi(footsteps):
    assert(len(footsteps) == NUM_REQUIRED_WALK_STEPS+2)
    walk_param_msg = drc.atlas_behavior_walk_params_t()
    walk_param_msg.num_required_walk_steps = NUM_REQUIRED_WALK_STEPS
    walk_param_msg.step_queue = [drc.atlas_step_data_t() for j in range(NUM_REQUIRED_WALK_STEPS)]
    walk_param_msg.use_relative_step_height = 0
    walk_param_msg.use_demo_walk = 0
    for j in range(NUM_REQUIRED_WALK_STEPS):
        step_data = walk_param_msg.step_queue[j]
        step_data.step_index = j+1
        step_data.foot_index = footsteps[j+2].is_right_foot
        step_data.duration = 0.6
        step_data.position = footsteps[j+2].pos[:3]
        step_data.yaw = footsteps[j+2].pos[5]
        step_data.normal = ut.rpy2rotmat(footsteps[j+2].pos[3:6,0]) * np.matrix([[0],[0],[1]])
        step_data.swing_height = 0.05  # (0.05 is BDI default) footsteps[j+2].step_height
    return walk_param_msg


def main():
    lc = lcm.LCM()

    def handle_footstep_plan(channel, msg_data):
        msg = drc.footstep_plan_t.decode(msg_data)
        footsteps, options = bdi_step.footsteps.decode_footstep_plan(msg)
        if len(footsteps) != NUM_REQUIRED_WALK_STEPS + 2:
            msg = 'ERROR: Footstep plan must be exactly 4 steps for BDI translation'
            print msg
            ut.send_status(6,0,0,msg)
            return
        walk_param_msg = translate_footsteps_to_bdi(footsteps)
        lc.publish('ATLAS_WALK_PARAMS', walk_param_msg.encode())

    lc.subscribe('APPROVED_FOOTSTEP_PLAN', handle_footstep_plan)
    while True:
        lc.handle()


if __name__ == "__main__":
    main()
