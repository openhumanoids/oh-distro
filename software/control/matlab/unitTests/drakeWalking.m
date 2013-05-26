function drakeWalking(use_mex)

addpath(strcat(getenv('DRC_PATH'),'/control/matlab/frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

num_steps = 5;
step_length = 0.5;
step_time = 1.0;

options.floating = true;
options.dt = 0.002;
if (nargin>0) options.use_mex = use_mex;
else options.use_mex = false; end

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = 0.05;

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 1000*randn();
xstar(2) = 1000*randn();
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
[c,V] = ZMPtracker(limp,zmptraj);
comtraj = ZMPplanFromTracker(limp,com(1:2),zeros(2,1),zmptraj,c);

% time spacing of samples for IK
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

ctrl_data = SharedDataHandle(struct(...
  'A',[zeros(2),eye(2); zeros(2,4)],...
  'B',[zeros(2); eye(2)],...
  'C',[eye(2),zeros(2)],...
  'D',[],...
  'Qy',eye(2),...
  'R',zeros(2),...
  'S',V.S.eval(0),... % always a constant
  's1',V.s1,...
  's2',V.s2,...
  'x0',[zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',comtraj,...
  'lfoottraj',lfoottraj, ...
  'rfoottraj',rfoottraj,...
  'supptraj',supptraj,...
  'y0',zmptraj));

% instantiate QP controller
options.slack_limit = 30.0;
options.w = 0.01;
options.R = 1e-12*eye(nu);
input_names = r.getInputFrame.coordinates;
ankle_idx = ~cellfun(@isempty,strfind(input_names,'lax')) | ~cellfun(@isempty,strfind(input_names,'uay'));
ankle_idx = find(ankle_idx);
options.R(ankle_idx,ankle_idx) = 10*options.R(ankle_idx,ankle_idx); % soft ankles
options.lcm_foot_contacts = false;
options.full_body_opt = false;
options.use_mex = true;
qp = QPController(r,ctrl_data,options);
clear options;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% feedback PD block 
pd = WalkingPDBlock(r,ctrl_data);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T],x0);
playback(v,traj,struct('slider',true));

err = 0; % x,y error
for i=1:length(ts)
  x=traj.eval(ts(i));
  q=x(1:getNumDOF(r)); 
  com(:,i)=getCOM(r,q);
  err = err + sum(abs(comtraj.eval(ts(i)) - com(1:2,i)));
end

figure(2);
subplot(3,1,1);
plot(ts,com(1,:),'r');
subplot(3,1,2);
plot(ts,com(2,:),'r');
subplot(3,1,3); hold on;
plot(com(1,:),com(2,:),'r');

err
if err > num_steps*0.5
  error('drakeWalking unit test failed: error is too large');
end


end
