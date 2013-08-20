from __future__ import division
import numpy as np
import lcm
import drc
import py_drake_utils as ut
import bdi_step.footsteps

NUM_REQUIRED_WALK_STEPS = 4

class BDIWalkTranslator:
    def __init__(self):
        self.lc = lcm.LCM()
        self.bdi_step_queue = []
        self.delivered_index = None
        self.drift_from_plan = np.zeros((3,1))

    def handle_footstep_plan(self, channel, msg_data):
        print "Starting new footstep plan"
        msg = drc.footstep_plan_t.decode(msg_data)
        footsteps, opts = bdi_step.footsteps.decode_footstep_plan(msg)
        if len(footsteps) < NUM_REQUIRED_WALK_STEPS:
            msg = 'ERROR: Footstep plan must be at least 4 steps for BDI translation'
            print msg
            ut.send_status(6,0,0,msg)
            return

        self.bdi_step_queue = [BDIWalkTranslator.footstep_to_step_data(f,j+1) for j,f in enumerate(footsteps)]
        self.send_walk_params(1)

    def handle_atlas_status(self, channel, msg_data):
        if self.delivered_index is None:
            return
        msg = drc.atlas_status_t.decode(msg_data)
        index_needed = msg.walk_feedback.next_step_index_needed
        if index_needed > (self.delivered_index + 1) and len(self.bdi_step_queue) >= (index_needed + 4):
            print "Handling request for next step: {:d}".format(index_needed)
            # self.update_drift(msg.walk_feedback.step_queue_saturated)
            self.send_walk_params(index_needed-1)

    def update_drift(self,step_queue):
        print "Updating drift calculation"
        planned = np.reshape(self.bdi_step_queue[self.delivered_index-1].position,(3,1))
        returned = np.reshape(step_queue[0].position,(3,1))
        print "planned:", planned
        print "returned:", returned
        self.drift_from_plan = returned - planned
        for s in self.bdi_step_queue[self.delivered_index:]:
            s.position[:2,0] += self.drift_from_plan[:2,0]

    def send_walk_params(self,step_index):
        walk_param_msg = drc.atlas_behavior_walk_params_t()
        walk_param_msg.num_required_walk_steps = NUM_REQUIRED_WALK_STEPS
        walk_param_msg.use_relative_step_height = 1  # as of Atlas 2.5.0 this flag is disabled and always acts as if it's set to 1
        walk_param_msg.use_demo_walk = 0
        walk_param_msg.step_queue = self.bdi_step_queue[step_index-1:step_index+3]
        self.lc.publish('ATLAS_WALK_PARAMS', walk_param_msg.encode())
        self.delivered_index = walk_param_msg.step_queue[0].step_index
        print "Sent walk params for step indices {:d} through {:d}".format(walk_param_msg.step_queue[0].step_index, walk_param_msg.step_queue[-1].step_index)

    def run(self):
        print "Ready to start translating footstep plans"
        self.lc.subscribe('APPROVED_FOOTSTEP_PLAN', self.handle_footstep_plan)
        self.lc.subscribe('ATLAS_STATUS', self.handle_atlas_status)
        while True:
            self.lc.handle()

    @staticmethod
    def footstep_to_step_data(footstep, step_index, duration=0.6,
                              use_relative_step_height=True):
        step_data = drc.atlas_step_data_t()
        step_data.step_index = step_index
        step_data.foot_index = footstep.is_right_foot
        step_data.duration = duration
        step_data.position = footstep.pos[:3]
        if use_relative_step_height:
            step_data.position[2] = 0
        step_data.yaw = footstep.pos[5]
        step_data.normal = ut.rpy2rotmat(footstep.pos[3:6,0]) * np.matrix([[0],[0],[1]])
        step_data.swing_height = footstep.step_height
        return step_data


def main():
    t = BDIWalkTranslator()
    t.run()

if __name__ == "__main__":
    main()
