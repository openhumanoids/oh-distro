options.floating = true;
options.dt = 0.001;

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

request = drc.footstep_plan_request_t();
request.utime = 0;

fp = load(strcat(getenv('DRC_PATH'), '/control/matlab/data/atlas_fp.mat'));
request.initial_state = r.getStateFrame().lcmcoder.encode(0, fp.xstar);

request.goal_pos = drc.position_3d_t();
request.goal_pos.translation = drc.vector_3d_t();
request.goal_pos.translation.x = 2.0;
request.goal_pos.translation.y = 0;
request.goal_pos.translation.z = 0;
request.goal_pos.rotation = drc.quaternion_t();
request.goal_pos.rotation.w = 1.0;
request.goal_pos.rotation.x = 0;
request.goal_pos.rotation.y = 0;
request.goal_pos.rotation.z = 0;

request.num_goal_steps = 0;

request.num_existing_steps = 0;

request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 10;
request.params.min_num_steps = 0;
request.params.min_step_width = 0.18;
request.params.nom_step_width = 0.26;
request.params.max_step_width = 0.35;
request.params.nom_forward_step = 0.2;
request.params.max_forward_step = 0.35;
request.params.ignore_terrain = true;
request.params.force_to_sticky_feet = false;
request.params.planning_mode = drc.footstep_plan_params_t.MODE_AUTO;
request.params.behavior = drc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING;
request.params.map_command = 0;
request.params.leading_foot = drc.footstep_plan_params_t.LEAD_AUTO;

request.default_step_params = drc.footstep_params_t();
request.default_step_params.utime = 0;
request.default_step_params.step_speed = 1.0;
request.default_step_params.step_height = 0.05;
request.default_step_params.bdi_step_duration = 0;
request.default_step_params.bdi_sway_duration = 0;
request.default_step_params.bdi_lift_height = 0;
request.default_step_params.bdi_toe_off = drc.atlas_behavior_step_action_t.TOE_OFF_ENABLE; 
request.default_step_params.bdi_knee_nominal = 0;
request.default_step_params.bdi_max_body_accel = 0;
request.default_step_params.bdi_max_foot_vel = 0;
request.default_step_params.bdi_sway_end_dist = 0.02;
request.default_step_params.bdi_step_end_dist = 0.02;
request.default_step_params.mu = 1.0;


p = StatelessFootstepPlanner(r);
p.plan_footsteps(request);
