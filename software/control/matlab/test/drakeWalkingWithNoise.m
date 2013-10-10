function drakeWalkingWithNoise()
% tests forward walking on flat terrain with state noise, actuator
% delays, and inertial/damping errors

addpath(fullfile(getDrakePath,'examples','ZMP'));

plot_comtraj = false;
navgoal = [randn();0.5*randn();0;0;0;pi*randn()];

% construct robot model---one for simulation, one for control (to test
% model discrepencies)
options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
v = r.constructVisualizer;
v.display_dt = 0.05;

% ******************* BEGIN ADJUSTABLE ************************************
% *************************************************************************
options.inertia_error = 0.05; % standard deviation for inertia noise (percentage of true inertia)
options.damping_error = 0.05; % standard deviation for damping noise (percentage of true joint damping)
% ******************* END ADJUSTABLE **************************************

rctrl = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = r.setInitialState(xstar);
rctrl = rctrl.setInitialState(xstar);

nq = getNumDOF(r);
nu = getNumInputs(r);

x0 = xstar;
q0 = x0(1:nq);

% create footstep and ZMP trajectories
step_options.max_num_steps = 100;
step_options.min_num_steps = 2;
step_options.step_height = 0.05;
step_options.step_speed = 0.75;
step_options.follow_spline = true;
step_options.right_foot_lead = true;
step_options.ignore_terrain = false;
step_options.nom_step_width = rctrl.nom_step_width;
step_options.nom_forward_step = rctrl.nom_forward_step;
step_options.max_forward_step = rctrl.max_forward_step;
step_options.goal_type = drc.walking_goal_t.GOAL_TYPE_CENTER;
step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;

footsteps = rctrl.createInitialSteps(x0, navgoal, step_options);
for j = 1:length(footsteps)
  footsteps(j).pos = r.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
end
[support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(rctrl, x0, footsteps,step_options);
link_constraints = buildLinkConstraints(rctrl, q0, foottraj);

ts = 0:0.1:zmptraj.tspan(end);
T = ts(end);

if plot_comtraj
  figure(2); 
  clf; 
  subplot(3,1,1); hold on;
  fnplt(zmptraj(1));
  fnplt(comtraj(1));
  subplot(3,1,2); hold on;
  fnplt(zmptraj(2));
  fnplt(comtraj(2));
  subplot(3,1,3); hold on;
  fnplt(zmptraj);
  fnplt(comtraj);
end

% compute s1,s2 derivatives for controller Vdot computation
s1dot = fnder(V.s1,1);
s2dot = fnder(V.s2,1);

ctrl_data = SharedDataHandle(struct(...
  'A',[zeros(2),eye(2); zeros(2,4)],...
  'B',[zeros(2); eye(2)],...
  'C',[eye(2),zeros(2)],...
  'Qy',eye(2),...
  'R',zeros(2),...
  'is_time_varying',true,...
  'S',V.S.eval(0),... % always a constant
  's1',V.s1,...
  's2',V.s2,...
  's1dot',s1dot,...
  's2dot',s2dot,...
  'x0',[zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',comtraj,...
  'link_constraints',link_constraints, ...
  'support_times',support_times,...
  'supports',[supports{:}],...
  'mu',1,...
  'ignore_terrain',true,...
  'y0',zmptraj));

% ******************* BEGIN ADJUSTABLE ************************************
% *************************************************************************
options.dt = 0.003;
options.slack_limit = 30.0;
options.w = 0.001;
options.lcm_foot_contacts = false;
options.debug = false;
options.use_mex = true;
% ******************* END ADJUSTABLE **************************************

% instantiate QP controller
qp = QPControlBlock(rctrl,ctrl_data,options);

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

% cascade footstep plan shift block
fs = FootstepPlanShiftBlock(rctrl,ctrl_data,options);
rnoisy = cascade(rnoisy,fs);

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(sys,rnoisy,[],[],ins,outs);
clear ins outs;

% feedback PD block 
pd = WalkingPDBlock(rctrl,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

qt = QTrajEvalBlock(rctrl,ctrl_data,options);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

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
rfoot_idx = findLinkInd(r,'r_foot');
lfoot_idx = findLinkInd(r,'l_foot');
rfoottraj = link_constraints(1).traj;
lfoottraj = link_constraints(2).traj;
for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); 
  kinsol = doKinematics(r,q);
  com(:,i)=getCOM(r,q);
  com_err = com_err + norm(comtraj.eval(ts(i)) - com(1:2,i))^2;
  
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
