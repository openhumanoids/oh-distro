function drakeWalkingDelayTest(delay_steps)
%NOTEST 

addpath(fullfile(getDrakePath,'examples','ZMP'));

num_steps = 5;
step_length = 0.5;
step_time = 1.0;

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

options.floating = true;
options.dt = 0.002;
options.use_mex = true;

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = 0.05;
r = r.setInitialState(xstar);

nq = getNumDOF(r);
nu = getNumInputs(r);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

% create desired ZMP trajectory
[zmptraj,lfoottraj,rfoottraj,support_times,supports] = ZMPandFootTrajectory(r,q0,num_steps,step_length,step_time);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
com = getCOM(r,kinsol);
limp = LinearInvertedPendulum(com(3));
% get COM traj from desired ZMP traj
[c,V,comtraj] = ZMPtracker(limp,zmptraj,struct('use_tvlqr',false,'com0',com(1:2)));

ts = 0:0.1:zmptraj.tspan(end);
T = ts(end);

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

link_constraints(1) = struct('link_ndx', r.findLinkInd('r_foot'), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', rfoottraj);
link_constraints(2) = struct('link_ndx', r.findLinkInd('l_foot'), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', lfoottraj);

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
  'x0',[zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',comtraj,...
  'link_constraints',link_constraints, ...
  'support_times',support_times,...
  'supports',[supports{:}],...
  'mu',1,...
  'ignore_terrain',false,...
  'y0',zmptraj));

% instantiate QP controller
options.dt = 0.004;
options.slack_limit = 30.0;
options.w = 0.01;
options.lcm_foot_contacts = false;
options.debug = false;
qp = QPControlBlock(r,ctrl_data,options);

% cascade qp controller with delay block
options.delay_steps = delay_steps;
options.use_input_frame = true;
delayblk = DelayBlock(r,options);
sys = cascade(qp,delayblk);

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(sys,r,[],[],ins,outs);
clear ins outs;

% feedback PD block 
pd = WalkingPDBlock(r,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

qt = QTrajEvalBlock(r,ctrl_data,options);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],[x0;zeros((delay_steps+1)*nu,1)]);
playback(v,traj,struct('slider',true));

com_err = 0; % x,y error
foot_err = 0; 
pelvis_sway = 0;
rfoot_idx = findLinkInd(r,'r_foot');
lfoot_idx = findLinkInd(r,'l_foot');
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

figure(2);
subplot(3,1,1);
plot(ts,com(1,:),'r');
subplot(3,1,2);
plot(ts,com(2,:),'r');
subplot(3,1,3); hold on;
plot(com(1,:),com(2,:),'r');

end
