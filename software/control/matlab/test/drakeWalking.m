function drakeWalking(options)
if nargin < 1
  options = struct();
end
options = applyDefaults(options, struct('use_bullet', false,...
                                        'use_angular_momentum', false,...
                                        'random_navgoal', false,...
                                        'hokuyo', false));

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5_fp.mat'));
if options.random_navgoal
  xstar(1) = randn();
  xstar(2) = randn();
  xstar(6) = pi*randn();
  navgoal = [xstar(1)+rand();xstar(2)+randn();0;0;0;pi*randn()];
else
  navgoal = [1;0;0;0;0;0]; % straight forward 1m
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
atlas_options = struct('floating', true,...
                       'ignore_friction', true,...
                       'dt', 0.001,...
                       'atlas_version', 5,...
                       'enable_fastqp', false,...
                       'use_bullet', options.use_bullet,...
                       'hokuyo', options.hokuyo,...
                       'visualize', false);
r = DRCAtlas([],atlas_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

nq = getNumPositions(r);

x0 = xstar;

% create footstep and ZMP trajectories
footstep_planner = StatelessFootstepPlanner();
request = drc.footstep_plan_request_t();
request.utime = 0;
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.goal_pos = encodePosition3d(navgoal);
request.num_goal_steps = 0;
request.num_existing_steps = 0;
request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 12;
request.params.min_num_steps = 2;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.24;
request.params.max_step_width = 0.3;
request.params.nom_forward_step = 0.5;
request.params.max_forward_step = 0.5;
request.params.nom_upward_step = 0.25;
request.params.nom_downward_step = 0.25;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_mode = drc.footstep_plan_params_t.HORIZONTAL_PLANE;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.4;
request.default_step_params.drake_min_hold_time = 0.75;
request.default_step_params.step_height = 0.05;
request.default_step_params.mu = 1.0;
request.default_step_params.drake_instep_shift = 0.0;
request.default_step_params.constrain_full_foot_pose = true;

footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
walking_plan = walking_planner.plan_walking(r, request, true);
walking_ctrl_msg = walking_planner.plan_walking(r, request, false);
walking_ctrl_data = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(walking_ctrl_msg, r);

% No-op: just make sure we can cleanly encode and decode the plan as LCM
tic;
walking_ctrl_data = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(DRCQPLocomotionPlan.toLCM(walking_ctrl_data), r);
fprintf(1, 'control data lcm code/decode time: %f\n', toc);
walking_ctrl_data = QPLocomotionPlanCPPWrapper(walking_ctrl_data);

% plot walking traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');
ts = walking_plan.ts;
for i=1:length(ts)
  lcmgl.glColor3f(0, 0, 1);
  lcmgl.sphere([walking_ctrl_data.settings.comtraj.eval(ts(i));0], 0.01, 20, 20);
  lcmgl.glColor3f(0, 1, 0);
  lcmgl.sphere([walking_ctrl_data.settings.zmptraj.eval(ts(i));0], 0.01, 20, 20);
end
lcmgl.switchBuffers();

planeval = bipedControllers.BipedPlanEval(r, walking_ctrl_data);
param_sets = atlasParams.getDefaults(r);
if options.use_angular_momentum
  param_sets.standing = StandingAngularMomentum(r);
  param_sets.walking = WalkingAngularMomentum(r);
end
control = bipedControllers.InstantaneousQPController(r, []);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

traj = simulate(sys, [0, walking_ctrl_data.settings.duration], walking_ctrl_data.settings.x0, struct('gui_control_interface', true));

playback(v,traj,struct('slider',true));

[com, rms_com] = r.plotWalkingTraj(r, traj, walking_ctrl_data);

if rms_com > length(footstep_plan.footsteps)*0.5
  error('drakeWalking unit test failed: error is too large');
  navgoal
end

end
