function drakeBalancing(use_mex,use_angular_momentum)

addpath(fullfile(getDrakePath,'examples','ZMP'));

% put robot in a random x,y,yaw position and balance for 3 seconds
visualize = true;

if (nargin<1); use_mex = true; end
if (nargin<2); use_angular_momentum = false; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

import atlasControllers.*;

options.floating = true;
options.dt = 0.002;
options.ignore_friction = 1;
options.atlas_version = 5;
r = DRCAtlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load(r.fixed_point_file);
xstar(1) = 10*randn();
xstar(2) = 10*randn();
xstar(6) = pi*randn();
%xstar(nq+1) = 0.1;
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

com = getCOM(r,kinsol);

% build TI-ZMP controller 
footidx = [findLinkId(r,'l_foot'), findLinkId(r,'r_foot')];
foot_pos = terrainContactPositions(r,kinsol,footidx);
comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
limp = LinearInvertedPendulum(com(3));
[~,V] = lqr(limp,comgoal);

foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));

pelvis_idx = findLinkId(r,'pelvis');

link_constraints(1).link_ndx = pelvis_idx;
link_constraints(1).pt = [0;0;0];
link_constraints(1).traj = ConstantTrajectory(forwardKin(r,kinsol,pelvis_idx,[0;0;0],1));
link_constraints(2).link_ndx = footidx(1);
link_constraints(2).pt = [0;0;0];
link_constraints(2).traj = ConstantTrajectory(forwardKin(r,kinsol,footidx(1),[0;0;0],1));
link_constraints(3).link_ndx = footidx(2);
link_constraints(3).pt = [0;0;0];
link_constraints(3).traj = ConstantTrajectory(forwardKin(r,kinsol,footidx(2),[0;0;0],1));


ctrl_data = QPControllerData(false,struct(...
  'acceleration_input_frame',atlasFrames.AtlasCoordinates(r),...
  'D',-com(3)/9.81*eye(2),...
  'Qy',eye(2),...
  'S',V.S,...
  's1',zeros(4,1),...
  's2',0,...
  'x0',[comgoal;0;0],...
  'u0',zeros(2,1),...
  'y0',comgoal,...
  'qtraj',x0(1:nq),...
  'support_times',0,...
  'supports',foot_support,...
  'link_constraints',link_constraints,...
  'mu',1.0,...
  'ignore_terrain',false,...
  'plan_shift',zeros(3,1),...
  'constrained_dofs',[findPositionIndices(r,'arm');findPositionIndices(r,'back');findPositionIndices(r,'neck')]));

% instantiate QP controller
options.debug = false;
options.use_mex = use_mex;

if use_angular_momentum
  options.Kp_ang = 1.0; % angular momentum proportunal feedback gain
  options.W_kdot = 1e-5*eye(3); % angular momentum weight
end

options.Kp_pelvis = [150; 150; 150; 200; 200; 200];
options.pelvis_damping_ratio = 0.6;
options.Kp_q = 150.0*ones(r.getNumPositions(),1);
options.q_damping_ratio = 0.6;

% construct QP controller and related control blocks
[qp,~,~,pelvis_controller,pd,options] = constructQPBalancingController(r,ctrl_data,options);

options.use_lcm=false;
options.contact_threshold = 0.002;
fc = FootContactBlock(r,ctrl_data,options);
qt = QTrajEvalBlock(r,ctrl_data,options);

sys = constructQPFeedbackCombination(r,qp,fc,pd,qt,[],[],pelvis_controller);

if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.05;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
x0(3) = 1.0; % drop it a bit

traj = simulate(sys,[0 3],x0);
if visualize
  playback(v,traj,struct('slider',true));
end

xf = traj.eval(traj.tspan(2));

err = norm(xf(1:6)-xstar(1:6))
if err > 0.02
  error('drakeBalancing unit test failed: error is too large');
end


end
