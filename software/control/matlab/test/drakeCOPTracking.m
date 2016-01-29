function drakeCOPTracking()

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','ZMP'));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
options.atlas_version = 5;
r = DRCAtlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

nq = getNumPositions(r);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_v5_fp.mat'));
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

rfoot_ind = r.findLinkId('r_foot');
lfoot_ind = r.findLinkId('l_foot');
foot_pos = terrainContactPositions(r,q0,[rfoot_ind, lfoot_ind]);
foot_center = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
zmptraj = zmptraj + foot_center;
zmptraj = zmptraj.setOutputFrame(desiredZMP);

com = getCOM(r,kinsol);
options.com0 = com(1:2);
options.build_control_objects = false;
zfeet = min(foot_pos(3,:));
[~,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);
comtraj = ExpPlusPPTrajectory(comtraj.breaks, comtraj.K, comtraj.A, comtraj.alpha, comtraj.gamma);

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

foot_support = RigidBodySupportState(r,[r.foot_body_id.right, r.foot_body_id.left]);
body_motions = BodyMotionData.empty();
for body_ind = [r.foot_body_id.right, r.foot_body_id.left, r.findLinkId('pelvis')]
  pose = forwardKin(r, kinsol, body_ind, [0;0;0], 2);
  body_motions(end+1) = BodyMotionData.from_body_xyzquat(body_ind, [0, inf], [pose, pose]);                                                 
end

plan_settings = QPLocomotionPlanSettings(r);
plan_settings.support_times = [0; zmptraj.tspan(end)];
plan_settings.supports = [foot_support, foot_support];
plan_settings.body_motions = body_motions;
plan_settings.zmptraj = zmptraj;
plan_settings.zmp_data.D = -com(3) / plan_settings.g * eye(2);
plan_settings.D_control = plan_settings.zmp_data.D;
plan_settings.V = V;
plan_settings.comtraj = comtraj;
plan_settings.duration = zmptraj.tspan(end) - 0.001;
plan_settings.gain_set = 'walking';
plan = QPLocomotionPlanCPPWrapper(plan_settings);

planeval = bipedControllers.BipedPlanEval(r, plan);
control = bipedControllers.InstantaneousQPController(r, []);
plancontroller = bipedControllers.BipedPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);

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

  kinsol = doKinematics(r,q,qd,struct('compute_JdotV',true));
  [com,J] = getCOM(r,kinsol);
  comJacDotTimesV = centerOfMassJacobianDotTimesV(r, kinsol, 1);
	% hardcoding D for ZMP output dynamics
	D = -1.04./9.81*eye(2);

	comdd = comJacDotTimesV + J * qdd;
	zmp = com(1:2) + D * comdd(1:2);
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
