import lcm
import drc
import bot_core
import time
import numpy as np

import py_drake_utils as ut
from bdi_step.footsteps import decode_footstep_plan, decode_deprecated_footstep_plan, encode_footstep_plan, FootGoal
from bdi_step.plotting import draw_swing
from bdi_step.utils import Behavior, gl, now_utime

NUM_REQUIRED_WALK_STEPS = 4

# Experimentally determined vector relating BDI's frame for foot position to ours. This is the xyz vector from the position of the foot origin (from drake forwardKin) to the BDI Atlas foot pos estimate, expressed in the frame of the foot.
ATLAS_FRAME_OFFSET = np.array([0.0400, 0.000, -0.0850])

def blank_step_spec():
    msg = drc.atlas_behavior_step_spec_t()
    msg.foot = drc.atlas_behavior_foot_data_t()
    msg.action = drc.atlas_behavior_step_action_t()
    return msg

def blank_walk_spec():
    msg = drc.atlas_behavior_walk_spec_t()
    msg.foot = drc.atlas_behavior_foot_data_t()
    msg.action = drc.atlas_behavior_walk_action_t()
    return msg

class Mode:
    translating = 0
    plotting = 1

class BDIStepTranslator(object):
    def __init__(self, mode=Mode.translating, safe=True):
        self.mode = mode
        self.safe = safe  # Don't send atlas behavior commands (to ensure that the robot never starts walking accidentally when running tests)
        self.lc = lcm.LCM()
        if self.mode == Mode.plotting:
            self.gl = gl
        else:
            self.gl = None
        self.bdi_step_queue_in = []
        self.delivered_index = None
        self.use_spec = True
        self.drift_from_plan = np.zeros((3,1))
        self.behavior = Behavior.BDI_STEPPING
        self.T_local_to_localbdi = bot_core.rigid_transform_t()
        self.T_local_to_localbdi.trans = np.zeros(3)
        self.T_local_to_localbdi.quat = ut.rpy2quat([0,0,0])

    def handle_bdi_transform(self, channel, msg):
        if isinstance(msg, str):
            msg = bot_core.rigid_transform_t.decode(msg)
        self.T_local_to_localbdi = msg

    def handle_footstep_plan(self, channel, msg):
        print "Starting new footstep plan"
        if isinstance(msg, str):
            try:
                msg = drc.deprecated_footstep_plan_t.decode(msg)
            except ValueError:
                msg = drc.footstep_plan_t.decode(msg)
        if isinstance(msg, drc.deprecated_footstep_plan_t):
            footsteps, opts = decode_deprecated_footstep_plan(msg)
        elif isinstance(msg, drc.footstep_plan_t):
            footsteps, opts = decode_footstep_plan(msg)
        else:
            raise ValueError("Can't decode footsteps: not a drc.footstep_plan_t or drc.deprecated_footstep_plan_t")

        behavior = opts['behavior']
        if behavior == Behavior.BDI_WALKING:
            # duration = 0.6
            if len(footsteps) < NUM_REQUIRED_WALK_STEPS+2:
                msg = 'ERROR: Footstep plan must be at least 4 steps for BDI walking translation'
                print msg
                ut.send_status(6,0,0,msg)
                return
        elif behavior != Behavior.BDI_STEPPING:
            m = "BDI step translator: Ignoring footstep plan without BDI_WALKING or BDI_STEPPING behavior"
            print m
            ut.send_status(6,0,0,m)
            return

        self.behavior = behavior

        if self.mode == Mode.plotting:
            self.draw(footsteps)
        else:
            self.bdi_step_queue_in = footsteps

            self.send_params(1)

            if not self.safe:
                m = "BDI step translator: Steps received; transitioning to {:s}".format("BDI_STEP" if self.behavior == Behavior.BDI_STEPPING else "BDI_WALK")
                print m
                ut.send_status(6,0,0,m)
                time.sleep(1)
                self.send_behavior()
            else:
                m = "BDI step translator: Steps received; in SAFE mode; not transitioning to {:s}".format("BDI_STEP" if self.behavior == Behavior.BDI_STEPPING else "BDI_WALK")
                print m
                ut.send_status(6,0,0,m)

    @property
    def bdi_step_queue_out(self):
        bdi_step_queue_out = [s.copy() for s in self.bdi_step_queue_in]

        for step in bdi_step_queue_out:
            # Transform to BDI coordinate frame
            T1 = ut.mk_transform(step.pos[:3], step.pos[3:])
            T2 = ut.mk_transform(self.T_local_to_localbdi.trans, ut.quat2rpy(self.T_local_to_localbdi.quat))
            T = T2.dot(T1)
            step.pos[:3] = T[:3,3]
            step.pos[3:] = ut.rotmat2rpy(T[:3,:3])

        self.lc.publish('BDI_ADJUSTED_FOOTSTEP_PLAN', encode_footstep_plan(bdi_step_queue_out).encode())

        for step in bdi_step_queue_out:
            # Express pos of the center of the foot, as expected by BDI
            R = ut.rpy2rotmat(step.pos[3:])
            offs = R.dot(ATLAS_FRAME_OFFSET)
            # import pdb; pdb.set_trace()
            step.pos[:3] += offs

        for i in reversed(range(2, len(bdi_step_queue_out))):
            bdi_step_queue_out[i].pos[2] -= bdi_step_queue_out[i-1].pos[2]

        return [s.to_bdi_spec(self.behavior, j+1) for j, s in enumerate(bdi_step_queue_out[2:])]

    def handle_atlas_status(self, channel, msg):
        if self.delivered_index is None or self.mode != Mode.translating:
            return
        if isinstance(msg, str):
            msg = drc.atlas_status_t.decode(msg)
        if self.behavior == Behavior.BDI_WALKING:
            index_needed = msg.walk_feedback.next_step_index_needed
            if (self.delivered_index + 1) < index_needed <= len(self.bdi_step_queue_in) - 4:
                print "Handling request for next step: {:d}".format(index_needed)
                self.send_params(index_needed-1)
        else:
            index_needed = msg.step_feedback.next_step_index_needed
            if self.delivered_index < index_needed <= len(self.bdi_step_queue_in) - 2:
                print "Handling request for next step: {:d}".format(index_needed)
                self.send_params(index_needed)

    def send_params(self,step_index,force_stop_walking=False):
        """
        Publish the next steppping footstep or up to the next 4 walking footsteps as needed.
        """
        assert self.mode == Mode.translating, "Translator in Mode.plotting mode is not allowed to send step/walk params"
        if self.behavior == Behavior.BDI_WALKING:
            walk_param_msg = drc.atlas_behavior_walk_params_t()
            walk_param_msg.num_required_walk_steps = NUM_REQUIRED_WALK_STEPS
            walk_param_msg.walk_spec_queue = self.bdi_step_queue_out[step_index-1:step_index+3]
            walk_param_msg.step_queue = [drc.atlas_step_data_t() for j in range(NUM_REQUIRED_WALK_STEPS)]  # Unused
            walk_param_msg.use_spec = True
            walk_param_msg.use_relative_step_height = 1  # as of Atlas 2.5.0 this flag is disabled and always acts as if it's set to 1
            walk_param_msg.use_demo_walk = 0
            if force_stop_walking:
                for step in walk_param_msg.walk_spec_queue:
                    step.step_index = -1
            self.lc.publish('ATLAS_WALK_PARAMS', walk_param_msg.encode())
            self.delivered_index = walk_param_msg.walk_spec_queue[0].step_index
            print "Sent walk params for step indices {:d} through {:d}".format(walk_param_msg.walk_spec_queue[0].step_index, walk_param_msg.walk_spec_queue[-1].step_index)
        elif self.behavior == Behavior.BDI_STEPPING:
            step_param_msg = drc.atlas_behavior_step_params_t()
            step_param_msg.desired_step = drc.atlas_step_data_t()  # Unused
            step_param_msg.desired_step_spec = self.bdi_step_queue_out[step_index-1]
            step_param_msg.use_relative_step_height = 1  # as of Atlas 2.5.0 this flag is disabled and always acts as if it's set to 1
            step_param_msg.use_demo_walk = 0
            step_param_msg.use_spec = True
            step_param_msg.desired_step = drc.atlas_step_data_t()  # Unused
            step_param_msg.desired_step_spec = self.bdi_step_queue_out[step_index-1]
            if force_stop_walking:
                step_param_msg.desired_step_spec.step_index = -1
            self.lc.publish('ATLAS_STEP_PARAMS', step_param_msg.encode())
            self.delivered_index = step_param_msg.desired_step_spec.step_index
            print "Sent step params for step index {:d}".format(step_param_msg.desired_step_spec.step_index)
        else:
            raise ValueError("Bad behavior value: {:s}".format(self.behavior))

    def send_behavior(self):
        command_msg = drc.atlas_behavior_command_t()
        command_msg.utime = now_utime()
        if self.behavior == Behavior.BDI_STEPPING:
            command_msg.command = "step"
        elif self.behavior == Behavior.BDI_WALKING:
            command_msg.command = "walk"
        else:
            raise ValueError("Tried to send invalid behavior to Atlas: {:s}".format(self.behavior))
        self.lc.publish("ATLAS_BEHAVIOR_COMMAND", command_msg.encode())

    def handle_stop_walking(self, channel, msg_data):
        """
        Generate a set of footsteps with -1 step indices, which will cause the BDI controller to switch to standing instead of continuing to walk
        """
        if self.behavior == Behavior.BDI_WALKING:
            n_steps = 6
        else:
            n_steps = 3
        footsteps = [FootGoal(pos=np.zeros((6)),
                              step_speed=0,
                              step_height=0,
                              step_id=0,
                              pos_fixed=np.zeros((6,1)),
                              is_right_foot=0,
                              is_in_contact=0,
                              bdi_step_duration=0,
                              bdi_sway_duration=0,
                              bdi_lift_height=0,
                              bdi_toe_off=0,
                              bdi_knee_nominal=0,
                              bdi_max_body_accel=0,
                              bdi_max_foot_vel=0,
                              bdi_sway_end_dist=-1,
                              bdi_step_end_dist=-1,
                              terrain_pts=np.matrix([]))] * n_steps

        self.bdi_step_queue_in = footsteps
        self.send_params(1, force_stop_walking=True)

        self.bdi_step_queue_in = []  # to prevent infinite spewing of -1 step indices
        self.delivered_index = None

    def run(self):
        if self.mode == Mode.translating:
            print "BDIStepTranslator running in robot-side translator mode"
            self.lc.subscribe('COMMITTED_FOOTSTEP_PLAN', self.handle_footstep_plan)
            self.lc.subscribe('STOP_WALKING', self.handle_stop_walking)
        else:
            print "BDIStepTranslator running in base-side plotter mode"
            self.lc.subscribe('CANDIDATE_BDI_FOOTSTEP_PLAN', self.handle_footstep_plan)
        self.lc.subscribe('ATLAS_STATUS', self.handle_atlas_status)
        self.lc.subscribe('LOCAL_TO_LOCAL_BDI', self.handle_bdi_transform)
        while True:
            self.lc.handle()

    def draw(self, footsteps):
        """
        Plot a rough guess of each swing foot trajectory, based on the BDI software manual's description of how swing_height and lift_height behave.
        """
        for j in range(len(footsteps)-2):
            st0 = footsteps[j].to_bdi_spec(self.behavior, 0)
            st1 = footsteps[j+2].to_bdi_spec(self.behavior, 0)
            is_stepping = self.behavior==Behavior.BDI_STEPPING
            if is_stepping:
                lift_height = st1.action.lift_height
            else:
                lift_height = None
            draw_swing(self.gl,
                       st0.foot.position,
                       st1.foot.position,
                       st1.action.swing_height,
                       is_stepping=is_stepping,
                       lift_height=lift_height)
        self.gl.switch_buffer()
