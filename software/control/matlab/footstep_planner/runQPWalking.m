function runQPWalking(goal_x, goal_y, goal_yaw)

if nargin < 3; goal_yaw = -1.57; end
if nargin < 2; goal_y = 1.0; end
if nargin < 1; goal_x = 0.5; end

options.floating = true;
options.dt = 0.002;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
r = r.setInitialState(xstar);
v = r.constructVisualizer;
v.display_dt = 0.05;

nq = getNumDOF(r);
nu = getNumInputs(r);
x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

% biped = Biped(r); % no longer necessary, since Atlas is a Biped
pose = [goal_x;goal_y;0;0;0;goal_yaw];

[rfoot, lfoot] = planFootsteps(r, x0, pose, struct('plotting', true, 'interactive', false));
[zmptraj,lfoottraj,rfoottraj,~,supptraj] = planZMPandFootTrajectory(r, q0, rfoot, lfoot, 0.8);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
com = getCOM(r,kinsol);
limp = LinearInvertedPendulum(com(3));
% get COM traj from desired ZMP traj
comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);

% time spacing of samples for IK
ts = linspace(0,zmptraj.tspan(end),100);
T = ts(end);

% create desired joint trajectory
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q0;
  
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

htraj = [];
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],rfoottraj.eval(t),lfoot_body,[0;0;0],lfoottraj.eval(t),options);
  else
    q = q0;
  end
  com = getCOM(r,q(:,i));
  htraj = [htraj com(3)];
  v.draw(t,q(:,i));
end
htraj = PPTrajectory(spline(ts,htraj));

figure(2); 
clf; 
subplot(3,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(3,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));
subplot(3,1,3); hold on;
fnplt(htraj);

limp = LinearInvertedPendulum(htraj);
[c, V] = ZMPtracker(limp,zmptraj);
zmpdata = SharedDataHandle(struct('V',V,'h',com(3),'c',c));

% instantiate QP controller
options.slack_limit = 10.0;
options.w = 1e-1;
options.R = 1e-12*eye(nu);
qp = QPController(r,zmpdata,options);

% desired configuration trajectory
qdes = setOutputFrame(PPTrajectory(spline(ts,q)),AtlasCoordinates(r));
pd = SimplePDController(r);
ins(1).system = 2;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
pd = mimoCascade(qdes,pd,[],ins,outs);
clear ins outs;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% walking foot support trajectory
ins(1).system = 2;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supptraj,sys,[],ins,outs);
clear ins outs;

% feedback PD trajectory controller 
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],[],outs);
clear outs;

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);
playback(v,traj,struct('slider',true));

for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); 
  com(:,i)=getCOM(r,q);
end

figure(2);
subplot(3,1,1);
plot(ts,com(1,:),'r');
subplot(3,1,2);
plot(ts,com(2,:),'r');
subplot(3,1,3);
plot(ts,com(3,:),'r');

keyboard;

end
