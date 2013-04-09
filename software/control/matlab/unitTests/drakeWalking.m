function drakeWalking

addpath('..');
addpath(fullfile(pwd,'../frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

num_steps = 10;
step_length = 0.5;
step_time = 1.0;

options.floating = true;
options.dt = 0.002;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = 0.05;

% set initial state to fixed point
load('../data/atlas_fp.mat');
r = r.setInitialState(xstar);

nq = getNumDOF(r);
nu = getNumInputs(r);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

% create desired ZMP trajectory
[zmptraj,lfoottraj,rfoottraj,supptraj] = ZMPandFootTrajectory(r,q0,num_steps,step_length,step_time);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
com = getCOM(r,kinsol);
limp = LinearInvertedPendulum(com(3));
% get COM traj from desired ZMP traj
comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);
[~,V] = ZMPtracker(limp,zmptraj);

% time spacing of samples for IK
ts = 0:0.1:zmptraj.tspan(end);
T = ts(end);

figure(2); 
clf; 
subplot(2,1,1); hold on;
fnplt(zmptraj(1));
fnplt(comtraj(1));
subplot(2,1,2); hold on;
fnplt(zmptraj(2));
fnplt(comtraj(2));

zmpdata = SharedDataHandle(struct('S',V.S,'h',com(3),'hddot',0, ...
                    'lfoottraj',lfoottraj,'rfoottraj',rfoottraj, ...
                    'comtraj',comtraj,'supptraj',supptraj,'ti_flag',false));

% instantiate QP controller
options.slack_limit = 10.0;
options.w = 1.0;
options.R = 1e-12*eye(nu);
qp = QPController(r,zmpdata,options);
clear options;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% feedback PD trajectory controller 
options.q_nom = q0;
pd = WalkingPDController(r,zmpdata,options);
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

err = 0; % x-y error
for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); 
  com(:,i)=getCOM(r,q);
  err = err + sum(abs(comtraj.eval(ts(i)) - com(1:2,i)));
end

figure(2);
subplot(2,1,1);
plot(ts,com(1,:),'r');
subplot(2,1,2);
plot(ts,com(2,:),'r');

err
if err > num_steps*0.5
  error('drakeWalking unit test failed: error is too large');
end


end
