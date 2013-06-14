function drakeWalking(use_mex)

addpath(strcat(getenv('DRC_PATH'),'/control/matlab/frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

num_steps = 5;
step_length = 0.5;
step_time = 1.0;

% set initial state to fixed point
d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/suppine_crawl.mat'));
x0 = d.x0; xstar = x0;

options.floating = true;
options.dt = 0.002;
if (nargin>0) options.use_mex = use_mex;
else options.use_mex = true; end

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','knuckle'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = 0.05;

r = r.setInitialState(xstar);

nq = getNumDOF(r);
nu = getNumInputs(r);


body_spec.body_ind = findLinkInd(r,'pelvis');
body_spec.pt = zeros(3,1);
foot_spec(1).body_ind = findLinkInd(r,'l_hand');
foot_spec(2).body_ind = findLinkInd(r,'r_hand');
foot_spec(3).body_ind = findLinkInd(r,'r_foot');
foot_spec(4).body_ind = findLinkInd(r,'l_foot');
foot_spec(1).contact_pt_ind = 1;  % removed all but knuckle above
foot_spec(2).contact_pt_ind = 1;
foot_spec(3).contact_pt_ind = 1;
foot_spec(4).contact_pt_ind = 2;
options.direction = 0;
options.step_length = .2;
options.gait = 2;
options.draw = false;

[q_traj,support_times,supports,V,comtraj,zmptraj,link_constraints] = crawlingPlan(r,x0,body_spec,foot_spec,options)
%qdot_traj = fnder(q_traj);
%qddot_traj = setOutputFrame(fnder(qdot_traj),AtlasCoordinates(r));

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
  'Qy',eye(2),...
  'R',zeros(2),...
  'is_time_varying',true,...
  'S',V.S.eval(0),... % always a constant
  's1',V.s1,...
  's2',0,... %V.s2,...
  'x0',[zmptraj.eval(zmptraj.tspan(end));0;0],...
  'u0',zeros(2,1),...
  'qtraj',q_traj,...
  'comtraj',comtraj,...
  'link_constraints',link_constraints, ...
  'support_times',support_times,...
  'supports',supports,...
  'mu',1,...
  'ignore_terrain',true,...
  'y0',zmptraj));

% instantiate QP controller
options.slack_limit = 30.0;
options.w = 0.01;
options.R = 1e-12*eye(nu);
input_names = r.getInputFrame.coordinates;
options.lcm_foot_contacts = false;
options.full_body_opt = true;
options.debug = false;

qp = QPController(r,ctrl_data,options);
clear options;

sys = r;

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,sys,[],[],ins,outs);
clear ins outs;

% feedback PD block 
walkingPDoptions.q_nom = xstar(1:nq);
pd = WalkingPDBlock(r,ctrl_data);
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(pd,sys,[],[],ins,outs);
clear ins outs;

setField(ctrl_data,'qtraj',q_traj);

qt = QTrajEvalBlock(r,ctrl_data);
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qt,sys,[],[],[],outs);

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,zmptraj.tspan,x0);
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
  error('drakeCrawling unit test failed: error is too large');
end


end
