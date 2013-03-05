function runQPWalking

options.floating = true;
options.dt = 0.002;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
nq = getNumDOF(r);
x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);
r = r.setInitialState(xstar);

biped = Biped(r);
poses = [0.25;0.5;0;0;0;0];

[rfoot, lfoot] = planFootsteps(biped, x0, poses, struct('plotting', true, 'interactive', false));
[zmptraj,lfoottraj,rfoottraj,~,supptraj] = planZMPandFootTrajectory(biped, q0, rfoot, lfoot, 0.8);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

v = r.constructVisualizer;
v.display_dt = 0.05;

com_ctrl = SimpleZMPTracker(r,x0,zmptraj);

options.support_accel = 'constraint';
if 1 % use cplex input formulation
  options.w = [1.0;1.0];
  qp = QPController(r,options);
else % use cvxgen CL formulation
  qp = QPControllerCVX(r,com_ctrl.V,com_ctrl.com0(3),1.0,zeros(getNumInputs(r)),options);
end

ts = linspace(0,zmptraj.tspan(end),150);
T = ts(end);

% get corresponding COM trajectory
com0 = getCOM(r,kinsol);
limp = LinearInvertedPendulum(com0(3));
comtraj = ZMPplanner(limp,com0(1:2),[0;0],zmptraj);

figure(2); 
clf; 
subplot(2,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(2,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));

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

for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],rfoottraj.eval(t),lfoot_body,[0;0;0],lfoottraj.eval(t),options);
  else
    q = q0;
  end
  v.draw(t,q(:,i));
end
qdes = setOutputFrame(PPTrajectory(spline(ts,q)),AtlasCoordinates(r));
pd = SimplePDController(r);

ins(1).system = 2;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
pd = mimoCascade(qdes,pd,[],ins,outs);
clear ins outs;

ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
ins(3).system = 1;
ins(3).input = 3;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supptraj,sys,[],ins,outs);
clear ins outs;

ins(1).system = 2;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(com_ctrl,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);
playback(v,traj,struct('slider',true));

for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); qd=x(getNumDOF(r)+1:end);
  com(:,i)=getCOM(r,q);
end

figure(2);
subplot(2,1,1);
plot(ts,com(1,:),'r');
subplot(2,1,2);
plot(ts,com(2,:),'r');

keyboard;

end
