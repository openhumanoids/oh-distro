function drakeReaching(noisy)

if nargin < 1
  noisy=false;
end

addpath(fullfile(getDrakePath,'examples','ZMP'));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = DRCAtlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(r.fixed_point_file);;
r = r.setInitialState(xstar);

if noisy
  % construct control model to test model discrepencies

  options.inertia_error = 0.15; % standard deviation for inertia noise (percentage of true inertia)
  options.damping_error = 0.1; % standard deviation for damping noise (percentage of true joint damping)

  rctrl = DRCAtlas([],options);
  % set initial state to fixed point
  rctrl = rctrl.setInitialState(xstar);
else
  rctrl = r;
end

nq = getNumPositions(r);
nu = getNumInputs(r);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% build TI-ZMP controller 
footidx = [findLinkId(r,'r_foot'), findLinkId(r,'l_foot')];
foot_pos = terrainContactPositions(r,kinsol,footidx); 
comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
limp = LinearInvertedPendulum(com(3));
[~,V] = lqr(limp,comgoal);

foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));
  
% generate manip plan
rhand_ind = findLinkId(r,'r_hand');
%rhand_ee = EndEffector(r,'atlas',rhand_ind,[0;0;0],'RHAND_OBS',false);
lhand_ind = findLinkId(r,'l_hand');
%lhand_ee = EndEffector(r,'atlas',lhand_ind,[0;0;0],'LHAND_OBS',false);
rhand_pos = forwardKin(r,kinsol,rhand_ind,[0;0;0],1);
lhand_pos = forwardKin(r,kinsol,lhand_ind,[0;0;0],1);
rhand_goal = rhand_pos(1:3) + [0.1+0.1*rand(); 0.05*randn(); 0.1+0.5*rand()];
lhand_goal = lhand_pos(1:3) + [0.1+0.1*rand(); 0.05*randn(); 0.1+0.5*rand()];

rfoot_ind = findLinkId(r,'r_foot');
lfoot_ind = findLinkId(r,'l_foot');
rfoot_pos = forwardKin(r,kinsol,rfoot_ind,[0;0;0],1);
lfoot_pos = forwardKin(r,kinsol,lfoot_ind,[0;0;0],1);

cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumPositions));
options.quastiStaticFlag = true;

% time spacing of samples for IK
T = 4;
ts = 0:0.1:T;

rhand_traj = PPTrajectory(spline([0 T],[rhand_pos(1:3) rhand_goal]));
lhand_traj = PPTrajectory(spline([0 T],[lhand_pos(1:3) lhand_goal]));
q = zeros(r.getNumPositions,length(ts));
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    rfoot_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,rfoot_ind,[0;0;0],rfoot_pos(1:3),rfoot_pos(1:3)),...
      constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldEulerConstraintType,...
      r.getMexModelPtr,rfoot_ind,rfoot_pos(4:6),rfoot_pos(4:6))};
    lfoot_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,lfoot_ind,[0;0;0],lfoot_pos(1:3),lfoot_pos(1:3)),...
      constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldEulerConstraintType,...
      r.getMexModelPtr,lfoot_ind,lfoot_pos(4:6),lfoot_pos(4:6))};
    rhand_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,rhand_ind,[0;0;0],rhand_traj.eval(t),rhand_traj.eval(t))};
    lhand_cnst = {constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldPositionConstraintType,...
      r.getMexModelPtr,lhand_ind,[0;0;0],lhand_traj.eval(t),lhand_traj.eval(t))};
    ikoptions = IKoptions(r);
    ikoptions = ikoptions.setQ(options.Q);
    q(:,i) = inverseKin(r,q(:,i-1),q(:,i-1), ...
      rfoot_cnst{:},lfoot_cnst{:},rhand_cnst{:},lhand_cnst{:},ikoptions);
      
%     q(:,i) = inverseKin(r,q(:,i-1), ...
%       rfoot_ind,[0;0;0],rfoot_pos, ...
%       lfoot_ind,[0;0;0],lfoot_pos, ...
%       rhand_ind,[0;0;0],rhand_traj.eval(t), ...
%       lhand_ind,[0;0;0],lhand_traj.eval(t),options);
  else
    q = q0;
  end
end

qtraj = PPTrajectory(pchip(ts,q));
plan_settings = QPLocomotionPlanSettings.fromQuasistaticQTraj(r, qtraj);
plan_settings.gain_set = 'manip';
plan = QPLocomotionPlanCPPWrapper(plan_settings);
planeval = bipedControllers.BipedPlanEval(r, plan);
control = bipedControllers.InstantaneousQPController(r, drcAtlasParams.getDefaults(r, true));
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);

if noisy
  options.delay_steps = 0;
else
  options.delay_steps = 1;
end
options.use_input_frame = true;
% cascade qp controller with delay block
delayblk = DelayBlock(r,options);
sys = cascade(plancontroller,delayblk);

if noisy
  options.deadband = 0.01 * r.umax; 
else
  options.deadband = 0;
end
% cascade qp controller with deadband block
dblk = DeadbandBlock(r,options);
sys = cascade(sys,dblk);

options.noise_model = struct();
% position noise on upper body joints
joint_names = r.getStateFrame.coordinates(1:nq);
arm_joints = find(~cellfun(@isempty,strfind(joint_names,'arm')));
upper_joints = find(~cellfun(@isempty,strfind(joint_names,'arm')) | ...
    ~cellfun(@isempty,strfind(joint_names,'back')) | ...
    ~cellfun(@isempty,strfind(joint_names,'neck')));
options.noise_model(1).ind = upper_joints'; 
options.noise_model(1).type = 'white_noise';
options.noise_model(2).ind = arm_joints'; 
options.noise_model(2).type = 'fixed_bias';
options.noise_model(3).ind = arm_joints'; 
options.noise_model(3).type = 'motion_bias';
if noisy
  options.noise_model(1).params = struct('std',0.001);
  options.noise_model(2).params = struct('bias',0);
%   options.noise_model(2).params = struct('bias',0.0001*randn(length(arm_joints),1));
  options.noise_model(3).params = struct('bias',0.02);
else
  options.noise_model(1).params = struct('std',0);
  options.noise_model(2).params = struct('bias',0);
  options.noise_model(3).params = struct('bias',0);
end
% velocity noise
options.noise_model(4).ind = (nq+upper_joints)';
options.noise_model(4).type = 'white_noise';
if noisy
  options.noise_model(4).params = struct('std',0.005);
else
  options.noise_model(4).params = struct('std',0);
end
% cascade robot with noise block
noiseblk = StateCorruptionBlock(r,options);
rnoisy = cascade(r,noiseblk);

sys = feedback(rnoisy, sys);

v = r.constructVisualizer;
v.display_dt = 0.05;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

traj = simulate(sys,[0 1.25*T],[x0;0*x0;zeros((options.delay_steps+1)*nu,1)]);

x=traj.eval(traj.tspan(end));
q=x(1:getNumPositions(r)); 
kinsol = doKinematics(r,q);
rhand_pos = forwardKin(r,kinsol,rhand_ind,[0;0;0]);
lhand_pos = forwardKin(r,kinsol,lhand_ind,[0;0;0]);

rhand_err = norm(rhand_goal-rhand_pos)
lhand_err = norm(lhand_goal-lhand_pos)

playback(v,traj,struct('slider',true));

if rhand_err > 0.03 || lhand_err > 0.03
  error('drakeReaching unit test failed: end effector steady state error too large.');
end
end
