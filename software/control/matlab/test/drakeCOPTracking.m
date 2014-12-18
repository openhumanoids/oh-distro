function drakeCOPTracking(use_mex)

addpath(fullfile(getDrakePath,'examples','ZMP'));

if (nargin<1); use_mex = true; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 0;%1*randn();
xstar(2) = 0;%1*randn();
xstar(6) = 0;%pi/2*randn();
%xstar(nq+1) = 0.1;
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

T = 20;
if 0
  % create figure 8 zmp traj
  dt = 0.01;
  ts = 0:dt:T;
  nt = T/dt;
  radius = 0.05; % 8 loop radius
  zmpx = [radius*sin(4*pi/T * ts(1:nt/2)), radius*sin(4*pi/T * ts(1:nt/2+1))];
  zmpy = [radius-radius*cos(4*pi/T * ts(1:nt/2)), -radius+radius*cos(4*pi/T * ts(1:nt/2+1))];
else
  % rectangle
  h=0.04; % height/2
  w=0.1; % width/2
  zmpx = [0 h h -h -h 0];
  zmpy = [0 w -w -w w 0];
  ts = [0 T/5 2*T/5 3*T/5 4*T/5 T];
end

zmpknots = [zmpx;zmpy;0*zmpx];
R = rpy2rotmat([0;0;x0(6)]);
zmpknots = R*zmpknots;
zmptraj = PPTrajectory(foh(ts,zmpknots(1:2,:)));

rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');
foot_pos = terrainContactPositions(r,q0,[rfoot_ind, lfoot_ind]);
foot_center = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
zmptraj = zmptraj + foot_center;
zmptraj = zmptraj.setOutputFrame(desiredZMP);

com = getCOM(r,kinsol);
options.com0 = com(1:2);
zfeet = min(foot_pos(3,:));
[~,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);

% plot zmp/com traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'zmp-traj');
ts = 0:0.1:T;
for i=1:length(ts)
  lcmgl.glColor3f(0, 1, 0);
	lcmgl.sphere([zmptraj.eval(ts(i));0], 0.01, 20, 20);
  lcmgl.glColor3f(1, 1, 0);
	lcmgl.sphere([comtraj.eval(ts(i));0], 0.01, 20, 20);
end
lcmgl.switchBuffers();

foot_support = RigidBodySupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));
link_constraints = struct('link_ndx',{}, 'pt', {}, 'min_traj', {}, 'max_traj', {}, 'traj', {});
for f = {'right', 'left'}
  foot = f{1};
  frame_id = r.foot_frame_id.(foot);
  body_ind = r.getFrame(frame_id).body_ind;
  link_constraints(end+1) = struct('link_ndx', body_ind, 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', ConstantTrajectory(forwardKin(r, kinsol, body_ind, [0;0;0], 1)));
end

ctrl_data = QPControllerData(true,struct(...
  'acceleration_input_frame',drcFrames.AtlasCoordinates(r),...
  'D',-com(3)/9.81*eye(2),...
  'Qy',eye(2),...
  'S',V.S.eval(0),...
  's1',V.s1,...
  's2',V.s2,...
  'x0',ConstantTrajectory([zmptraj.eval(T);0;0]),...
  'u0',ConstantTrajectory(zeros(2,1)),...
  'y0',zmptraj,...
  'qtraj',q0,...
  'support_times',0,...
  'supports',foot_support,...
  'mu',1.0,...
  'link_constraints',link_constraints,...
  'ignore_terrain',false,...
  'comtraj',comtraj,...
  'plan_shift',[0;0;0],...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));

% instantiate QP controller
options.slack_limit = 20;
options.w_qdd = 0.001*ones(nq,1);
options.W_kdot = zeros(3);
options.w_grf = 0;
options.w_slack = 0.001;
options.debug = false;
options.use_mex = use_mex;
options.contact_threshold = 0.002;
options.output_qdd = true;

% feedback QP controller with atlas
qp = QPController(r,{},ctrl_data,options);

ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins;

% feedback foot contact detector with QP/atlas
options.use_lcm=false;
options.contact_threshold = 0.002;
fc = FootContactBlock(r,ctrl_data,options);

ins(1).system = 2;
ins(1).input = 1;
sys = mimoFeedback(fc,sys,[],[],ins,outs);
clear ins;

% feedback PD trajectory controller
options.Kp = 80.0*ones(nq,1);
options.Kd = 8.0*ones(nq,1);
pd = IKPDBlock(r,ctrl_data,options);

ins(1).system = 1;
ins(1).input = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins;

qt = QTrajEvalBlock(r,ctrl_data);
sys = mimoFeedback(qt,sys,[],[],[],outs);

v = r.constructVisualizer;
v.display_dt = 0.05;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

xtraj = simulate(sys,[0 T],x0);
playback(v,xtraj,struct('slider',true));


ts = 0:0.01:T;

% plot tracking performance
alpha = 0.01;
zmpact = [];
for i=1:length(ts)
  x = xtraj.eval(ts(i));
  q = x(1:nq);
  qd = x(nq+(1:nq));

  if i==1
		qdd = 0*qd;
	else
		qdd = (1-alpha)*qdd_prev + alpha*(qd-qd_prev)/0.01;
  end
  qd_prev = qd;
	qdd_prev = qdd;

  kinsol = doKinematics(r,q,false,true);
  [com,J] = getCOM(r,kinsol);
	J = J(1:2,:);
	Jdot = forwardJacDot(r,kinsol,0);
  Jdot = Jdot(1:2,:);

	% hardcoding D for ZMP output dynamics
	D = -1.04./9.81*eye(2);

	comdd = Jdot * qd + J * qdd;
	zmp = com(1:2) + D * comdd;
	zmpact = [zmpact [zmp;0]];
end

nb = length(zmptraj.getBreaks());
zmpknots = reshape(zmptraj.eval(zmptraj.getBreaks()),2,nb);
zmpknots = [zmpknots; zeros(1,nb)];

zmpact = R'*zmpact;
zmpknots = R'*zmpknots;

figure(11);
plot(zmpact(2,:),zmpact(1,:),'r');
hold on;
plot(zmpknots(2,:),zmpknots(1,:),'g');
hold off;
axis equal;

end
