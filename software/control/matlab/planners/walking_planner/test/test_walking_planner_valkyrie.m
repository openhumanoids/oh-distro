%NOTEST
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
r = OHValkyrie();
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);


footstep_request = drc.footstep_plan_request_t();
footstep_request.utime = 0;

fixed_pt = r.loadFixedPoint();
footstep_request.initial_state = r.getStateFrame().lcmcoder.encode(0, fixed_pt);

footstep_request.goal_pos = drc.position_3d_t();
footstep_request.goal_pos.translation = drc.vector_3d_t();
footstep_request.goal_pos.translation.x = 2.0;
footstep_request.goal_pos.translation.y = 0;
footstep_request.goal_pos.translation.z = 0;
footstep_request.goal_pos.rotation = drc.quaternion_t();
footstep_request.goal_pos.rotation.w = 1.0;
footstep_request.goal_pos.rotation.x = 0;
footstep_request.goal_pos.rotation.y = 0;
footstep_request.goal_pos.rotation.z = 0;

footstep_request.num_goal_steps = 0;
footstep_request.num_existing_steps = 0;
footstep_request.params = drc.footstep_plan_params_t();
footstep_request.params.max_num_steps = 10;
footstep_request.params.min_num_steps = 0;
footstep_request.params.min_step_width = 0.25;
footstep_request.params.nom_step_width = 0.3;
footstep_request.params.max_step_width = 0.44;
footstep_request.params.nom_forward_step = 0.15;
footstep_request.params.max_forward_step = 0.3;
footstep_request.params.nom_upward_step = 0.25;
footstep_request.params.nom_downward_step = 0.15;
footstep_request.params.planning_mode = drc.footstep_plan_params_t.MODE_AUTO;
footstep_request.params.behavior = drc.footstep_plan_params_t.BEHAVIOR_BDI_STEPPING;
footstep_request.params.map_mode = drc.footstep_plan_params_t.FOOT_PLANE;
footstep_request.params.leading_foot = drc.footstep_plan_params_t.LEAD_LEFT;

footstep_request.default_step_params = drc.footstep_params_t();
footstep_request.default_step_params.utime = 0;
footstep_request.default_step_params.step_speed = 0.2;
footstep_request.default_step_params.drake_min_hold_time = 2.0;
footstep_request.default_step_params.step_height = 0.05;
footstep_request.default_step_params.constrain_full_foot_pose = true;
footstep_request.default_step_params.mu = 1.0;


fp = StatelessFootstepPlanner();
plan = fp.plan_footsteps(r, footstep_request);
tstep = plan.compute_step_timing(r);
plan_msg = plan.toLCM();

request = drc.walking_plan_request_t();
request.utime = 0;

request.initial_state = r.getStateFrame().lcmcoder.encode(0, fixed_pt);
request.new_nominal_state = request.initial_state;
request.use_new_nominal_state = false;
request.footstep_plan = plan_msg;

wp = StatelessWalkingPlanner();
% Compute walking trajectory
walking_plan = wp.plan_walking(r, request, true);
lc = lcm.lcm.LCM.getSingleton();
lc.publish('CANDIDATE_ROBOT_PLAN', walking_plan.toLCM());

% disp('Playing back plan at 10X speed');
% r = r.setTerrain(RigidBodyTerrain());
% v = r.constructVisualizer();
% xt = PPTrajectory(spline(walking_plan.ts/10, walking_plan.xtraj));
% xt = xt.setOutputFrame(r.getStateFrame());
% v.draw(0, fixed_pt);
% v.playback(xt);

% Compute walking controller
walking_plan = wp.plan_walking(r, request, false);
lc = lcm.lcm.LCM.getSingleton();
% lc.publish('CANDIDATE_ROBOT_PLAN', walking_plan.toLCM());
ts = linspace(walking_plan.zmptraj.tspan(1), walking_plan.zmptraj.tspan(2), 1000);
zmps = walking_plan.zmptraj.eval(ts);
plot(ts, zmps(1,:), ts, zmps(2,:))
legend('x', 'y')
