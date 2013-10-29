import random
import unittest
import drc
import lcm
from bdi_step.translator import BDIStepTranslator, Mode


class TestStepTranslation(unittest.TestCase):
    def generate_plan(self, behavior):
        plan = drc.footstep_plan_t()
        plan.utime = 0
        plan.robot_name = 'atlas'
        plan.num_steps = 10
        plan.is_new_plan = True
        plan.footstep_opts = drc.footstep_opts_t()
        plan.footstep_opts.ignore_terrain = True
        plan.footstep_opts.mu = True
        plan.footstep_opts.behavior = behavior

        for j in range(plan.num_steps):
            goal = drc.footstep_goal_t()
            goal.utime = 0
            goal.robot_name = 'atlas'
            goal.pos = drc.position_3d_t();
            goal.pos.translation = drc.vector_3d_t()
            if 1 <= j <= 2:  # this works in Python!
                goal.pos.translation.x = 0
                goal.pos.translation.y = 0
                goal.pos.translation.z = 0
                goal.pos.rotation = drc.quaternion_t()
                goal.pos.rotation.x = 0
                goal.pos.rotation.y = 0
                goal.pos.rotation.z = 0
                goal.pos.rotation.w = 1
            else:
                goal.pos.translation.x = 0.15 * j + (0.5 - random.random()) * 0.15
                goal.pos.translation.y = random.random() * 0.15 * j + (0.5 - random.random()) * 0.2
                goal.pos.translation.z = (0.5 - random.random()) * 0.2
                goal.pos.rotation = drc.quaternion_t()
                goal.pos.rotation.x = random.random()
                goal.pos.rotation.y = random.random()
                goal.pos.rotation.z = random.random()
                goal.pos.rotation.w = random.random()
            goal.step_speed = 1.5
            goal.step_height = random.random() * 0.25
            goal.id = j+1
            goal.is_right_foot = j % 2 == 0
            goal.is_in_contact = True
            goal.fixed_x = True
            goal.fixed_y = True
            goal.fixed_z = True
            goal.fixed_roll = True
            goal.fixed_pitch= True
            goal.fixed_yaw = True
            goal.bdi_step_duration = 2.0
            goal.bdi_sway_duration = 0
            goal.bdi_lift_height = random.random() * 0.25
            goal.bdi_toe_off = 0
            goal.bdi_knee_nominal = 0

            plan.footstep_goals.append(goal)
        return plan

    def test_stepping(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_STEPPING)
        lc = lcm.LCM()
        lc.publish('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

        translator = BDIStepTranslator()
        def handle_steps(channel, msg_data):
            msg = drc.atlas_behavior_step_params_t.decode(msg_data)
            self.assertAlmostEqual(msg.desired_step_spec.foot.position[0], 0.06)
            self.assertAlmostEqual(msg.desired_step_spec.foot.position[1], 0)
            self.assertAlmostEqual(msg.desired_step_spec.foot.position[2], 0)
            self.assertAlmostEqual(msg.desired_step_spec.foot.normal[0], 0)
            self.assertAlmostEqual(msg.desired_step_spec.foot.normal[1], 0)
            self.assertAlmostEqual(msg.desired_step_spec.foot.normal[2], 1)
            self.assertEqual(msg.use_spec, True)
        lc.subscribe('ATLAS_STEP_PARAMS', handle_steps)
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        lc.handle()

        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[0], 0.06)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[2], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[0], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[2], 1)
        self.assertEqual(translator.use_spec, True)

        plotter = BDIStepTranslator(mode=Mode.plotting)
        plotter.handle_footstep_plan('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

    def test_stepping_deprecated(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_STEPPING)
        lc = lcm.LCM()
        lc.publish('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

        translator = BDIStepTranslator(use_spec=False)
        def handle_steps(channel, msg_data):
            msg = drc.atlas_behavior_step_params_t.decode(msg_data)
            self.assertAlmostEqual(msg.desired_step.position[0], 0.06)
            self.assertAlmostEqual(msg.desired_step.position[1], 0)
            self.assertAlmostEqual(msg.desired_step.position[2], 0)
            self.assertAlmostEqual(msg.desired_step.normal[0], 0)
            self.assertAlmostEqual(msg.desired_step.normal[1], 0)
            self.assertAlmostEqual(msg.desired_step.normal[2], 1)
            self.assertEqual(msg.use_spec, False)
        lc.subscribe('ATLAS_STEP_PARAMS', handle_steps)
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        lc.handle()

        self.assertAlmostEqual(translator.bdi_step_queue[0].position[0], 0.06)
        self.assertAlmostEqual(translator.bdi_step_queue[0].position[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].position[2], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[0], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[2], 1)
        self.assertEqual(translator.use_spec, False)

        plotter = BDIStepTranslator(mode=Mode.plotting)
        plotter.handle_footstep_plan('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

    def test_walking(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_WALKING)

        lc = lcm.LCM()
        lc.publish('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

        translator = BDIStepTranslator()
        def handle_steps(channel, msg_data):
            msg = drc.atlas_behavior_walk_params_t.decode(msg_data)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.position[0], 0.06)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.position[1], 0)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.position[2], 0)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.normal[0], 0)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.normal[1], 0)
            self.assertAlmostEqual(msg.walk_spec_queue[0].foot.normal[2], 1)
            self.assertEqual(msg.use_spec, True)
        lc.subscribe('ATLAS_WALK_PARAMS', handle_steps)
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        lc.handle()

        plotter = BDIStepTranslator(mode=Mode.plotting)
        plotter.handle_footstep_plan('CANDIDATE_FOOTSTEP_PLAN', plan.encode())
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[0], 0.06)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.position[2], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[0], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].foot.normal[2], 1)
        self.assertEqual(translator.use_spec, True)

    def test_walking_deprecated(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_WALKING)
        lc = lcm.LCM()
        lc.publish('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

        translator = BDIStepTranslator(use_spec=False)
        def handle_steps(channel, msg_data):
            msg = drc.atlas_behavior_walk_params_t.decode(msg_data)
            self.assertAlmostEqual(msg.step_queue[0].position[0], 0.06)
            self.assertAlmostEqual(msg.step_queue[0].position[1], 0)
            self.assertAlmostEqual(msg.step_queue[0].position[2], 0)
            self.assertAlmostEqual(msg.step_queue[0].normal[0], 0)
            self.assertAlmostEqual(msg.step_queue[0].normal[1], 0)
            self.assertAlmostEqual(msg.step_queue[0].normal[2], 1)
            self.assertEqual(msg.use_spec, False)
        lc.subscribe('ATLAS_WALK_PARAMS', handle_steps)
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        lc.handle()

        plotter = BDIStepTranslator(mode=Mode.plotting)
        plotter.handle_footstep_plan('CANDIDATE_FOOTSTEP_PLAN', plan.encode())
        self.assertAlmostEqual(translator.bdi_step_queue[0].position[0], 0.06)
        self.assertAlmostEqual(translator.bdi_step_queue[0].position[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].position[2], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[0], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[1], 0)
        self.assertAlmostEqual(translator.bdi_step_queue[0].normal[2], 1)
        self.assertEqual(translator.use_spec, False)

    def test_stop_walking(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_STEPPING)
        translator = BDIStepTranslator()
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        translator.handle_stop_walking('STOP_WALKING', None)
        self.assertEqual(translator.bdi_step_queue[0].step_index, -1)

        translator = BDIStepTranslator()
        translator.handle_stop_walking('STOP_WALKING', None)
        self.assertEqual(translator.bdi_step_queue[0].step_index, -1)

    def test_atlas_status(self):
        plan = self.generate_plan(drc.footstep_opts_t.BEHAVIOR_BDI_STEPPING)
        lc = lcm.LCM()
        lc.publish('CANDIDATE_FOOTSTEP_PLAN', plan.encode())

        translator = BDIStepTranslator()
        translator.handle_footstep_plan('COMMITTED_FOOTSTEP_PLAN', plan.encode())
        self.assertEqual(translator.delivered_index, 1)

        status = drc.atlas_status_t()
        status.step_feedback = drc.atlas_step_feedback_t()
        status.step_feedback.next_step_index_needed = 2
        translator.handle_atlas_status('ATLAS_STATUS', status)
        self.assertEqual(translator.delivered_index, 2)


