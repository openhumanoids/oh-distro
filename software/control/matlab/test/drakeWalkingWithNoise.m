function drakeWalkingWithNoise()
%NOTEST
% tests forward walking on flat terrain with state noise, actuator
% delays, and inertial/damping errors

use_bullet = false;

addpath(fullfile(getDrakePath,'examples','ZMP'));

plot_comtraj = false;
%navgoal = [0.5*randn();0.5*randn();0;0;0;pi/2*randn()];
navgoal = [1;0;0;0;0;0];

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model---one for simulation, one for control (to test
% model discrepencies)
options.floating = true;
options.ignore_friction = true;
options.dt = 0.002;
r = DRCAtlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);
v = r.constructVisualizer;
v.display_dt = 0.05;

% ******************* BEGIN ADJUSTABLE ************************************
% *************************************************************************
options.inertia_error = 0.05; % standard deviation for inertia noise (percentage of true inertia)
options.damping_error = 0.05; % standard deviation for damping noise (percentage of true joint damping)
% ******************* END ADJUSTABLE **************************************

rctrl = DRCAtlas([],options);

% set initial state to fixed point
load(rctrl.fixed_point_file);
r = r.setInitialState(xstar);
rctrl = rctrl.setInitialState(xstar);

nq = getNumPositions(r);
nu = getNumInputs(r);

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
request.params.max_num_steps = 30;
request.params.min_num_steps = 2;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.24;
request.params.max_step_width = 0.3;
request.params.nom_forward_step = 0.2;
request.params.max_forward_step = 0.4;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_mode = drc.footstep_plan_params_t.FOOT_PLANE;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.5;
request.default_step_params.drake_min_hold_time = 2.0;
request.default_step_params.step_height = 0.05;
request.default_step_params.mu = 1.0;
request.default_step_params.constrain_full_foot_pose = false;

footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
walking_plan = walking_planner.plan_walking(r, request, true);
walking_ctrl_msg = walking_planner.plan_walking(r, request, false);
walking_ctrl_data = DRCQPLocomotionPlan.from_qp_locomotion_plan_t(walking_ctrl_msg, r);
walking_ctrl_data = QPLocomotionPlanCPPWrapper(walking_ctrl_data);

ts = walking_plan.ts;
T = ts(end);

% plot walking traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');
ts = walking_plan.ts;
for i=1:length(ts)
  lcmgl.glColor3f(0, 0, 1);
  lcmgl.sphere([walking_ctrl_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
  lcmgl.glColor3f(0, 1, 0);
  lcmgl.sphere([walking_ctrl_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
end
lcmgl.switchBuffers();

planeval = bipedControllers.BipedPlanEval(r, walking_ctrl_data);
param_sets = atlasParams.getDefaults(r);
control = bipedControllers.InstantaneousQPController(r, []);
qp = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);

% ******************* BEGIN ADJUSTABLE ************************************
% *************************************************************************
options.delay_steps = 1;
options.use_input_frame = true;
% ******************* END ADJUSTABLE **************************************

% cascade qp controller with delay block
delayblk = DelayBlock(r,options);
sys = cascade(qp,delayblk);

% ******************* BEGIN ADJUSTABLE ************************************
% *************************************************************************
options.noise_model = struct();
% position noise
options.noise_model(1).ind = (1:nq)';
options.noise_model(1).type = 'gauss_markov';
options.noise_model(1).params = struct('std',0.0001);
% velocity noise
options.noise_model(2).ind = (nq+(1:nq))';
options.noise_model(2).type = 'white_noise';
options.noise_model(2).params = struct('std',0.0025);
% ******************* END ADJUSTABLE **************************************

% cascade robot with noise block
noiseblk = StateCorruptionBlock(r,options);
rnoisy = cascade(r,noiseblk);

sys = feedback(rnoisy, sys);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],[x0;0*x0;zeros((options.delay_steps+1)*nu,1)]);
playback(v,traj,struct('slider',true));

com_err = 0; % x,y error
foot_err = 0;
pelvis_sway = 0;
rfoot_idx = findLinkId(r,'r_foot');
lfoot_idx = findLinkId(r,'l_foot');
rfoottraj = PPTrajectory(mkpp(walking_ctrl_data.link_constraints(1).ts, walking_ctrl_data.link_constraints(1).coefs, 6));
lfoottraj = PPTrajectory(mkpp(walking_ctrl_data.link_constraints(2).ts, walking_ctrl_data.link_constraints(2).coefs, 6));
for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumPositions(r));
  kinsol = doKinematics(r,q);
  com(:,i)=getCOM(r,q);
  com_err = com_err + norm(walking_ctrl_data.comtraj.eval(ts(i)) - com(1:2,i))^2;

  rfoot_pos = forwardKin(r,kinsol,rfoot_idx,[0;0;0]);
  rfoot_des = rfoottraj.eval(ts(i));
  lfoot_pos = forwardKin(r,kinsol,lfoot_idx,[0;0;0]);
  lfoot_des = lfoottraj.eval(ts(i));
  foot_err = foot_err + norm(rfoot_des(1:3) - rfoot_pos)^2 + norm(lfoot_des(1:3) - lfoot_pos)^2;

  pelvis_pos = forwardKin(r,kinsol,rfoot_idx,[0;0;0],1);
  pelvis_sway = pelvis_sway + sum(abs(pelvis_pos(4:6)));
end
com_err = sqrt(com_err / length(ts))
foot_err = sqrt(foot_err / length(ts))
pelvis_sway

if plot_comtraj
  figure(2);
  subplot(3,1,1);
  plot(ts,com(1,:),'r');
  subplot(3,1,2);
  plot(ts,com(2,:),'r');
  subplot(3,1,3); hold on;
  plot(com(1,:),com(2,:),'r');
end

if com_err > 0.15
  error('drakeWalking unit test failed: error is too large');
  navgoal
end

end
