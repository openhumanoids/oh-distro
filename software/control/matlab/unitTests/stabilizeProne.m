function stabilizeProne

test_ignore_states = false;

addpath(strcat(getenv('DRC_PATH'),'/control/matlab/frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

nq = getNumDOF(r);
nu = getNumInputs(r);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_prone.mat'));
%xstar(6) = pi*randn();
r = r.setInitialState(xstar);

x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

A = [zeros(2),eye(2); zeros(2,4)];
B = [zeros(2); eye(2)];
Q = eye(4);
R = 0.001*eye(2);

com = getCOM(r,kinsol);
comgoal=com(1:2);
ltisys = LinearSystem(A,B,[],[],[],[]);
[~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),Q,R);


l_knee_idx = r.findLinkInd('l_lleg');
cpts{1} = 1:size(getBodyContacts(r,l_knee_idx),2);
r_knee_idx = r.findLinkInd('r_lleg');
cpts{2} = 1:size(getBodyContacts(r,r_knee_idx),2);

l_hand_idx = r.findLinkInd('l_hand');
cpts{3} = 1:size(getBodyContacts(r,l_hand_idx),2);
r_hand_idx = r.findLinkInd('r_hand');
cpts{4} = 1:size(getBodyContacts(r,r_hand_idx),2);

% l_foot = r.findLink('l_foot');
% l_foot_idx = r.findLinkInd('l_foot');
% cpts{5} = l_foot.collision_group{find(strcmp(l_foot.collision_group_name,'toe'))}; 
% r_foot = r.findLink('r_foot');
% r_foot_idx = r.findLinkInd('r_foot');
% cpts{6} = r_foot.collision_group{find(strcmp(r_foot.collision_group_name,'toe'))}; 

% bodies = [l_knee_idx,r_knee_idx,l_hand_idx,r_hand_idx,l_foot_idx,r_foot_idx];
bodies = [l_knee_idx,r_knee_idx,l_hand_idx,r_hand_idx];

supports = SupportState(r,bodies,cpts);
 
if test_ignore_states
  xtraj = ConstantTrajectory(x0);
  options.ignore_states = [1 2 3]';
else
  xtraj = [];
end

ctrl_data = SharedDataHandle(struct(...
  'A',A,...
  'B',B,...
  'C',zeros(2,4),...
  'D',zeros(2,2),...
  'Qy',zeros(2),...
  'R',R,...
  'S',V.S,...
  's1',zeros(4,1),...
  's2',0,...
  'x0',[comgoal;0;0],...
  'u0',zeros(2,1),...
  'y0',zeros(2,1),...
  'qtraj',q0,...
  'mu',1,...
  'trans_drift',[0;0;0],...
  'ignore_terrain',false,...
  'is_time_varying',false,...
  'support_times',0,...
  'xtraj',xtraj,...
  'supports',supports));          

% instantiate QP controller
options.slack_limit = 30.0;
options.w = 0.01;
options.R = 1e-12*eye(nu);
options.lcm_foot_contacts = false;
options.full_body_opt = true;
options.use_mex = false;
options.debug = false;

qp = QPController(r,ctrl_data,options);
clear options;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,r,[],[],ins,outs);
clear ins outs;

% feedback PD trajectory controller 
pd = SimplePDBlock(r,ctrl_data);
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




v = r.constructVisualizer;
v.display_dt = 0.05;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

x0(3) = x0(3) + 0.2; 

traj = simulate(sys,[0 2],x0);
playback(v,traj,struct('slider',true));


end
