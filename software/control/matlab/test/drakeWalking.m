function drakeWalking(use_mex,use_bullet)

addpath(fullfile(getDrakePath,'examples','ZMP'));

plot_comtraj = true;
navgoal = [1.0;0;0;0;0;0];
%navgoal = [randn();0.5*randn();0;0;0;pi*randn()];

% construct robot model
options.floating = true;
options.dt = 0.002;
if (nargin>0) options.use_mex = use_mex;
else options.use_mex = true; end
if (nargin<2) 
  use_bullet = false; % test walking with the controller computing pairwise contacts using bullet
end

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 0*randn();
xstar(2) = 0*randn();
r = r.setInitialState(xstar);

if use_bullet
  r_bullet = RigidBodyManipulator();
  r_bullet = addRobotFromURDF(r_bullet,strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),[0;0;0],[0;0;0],options);
  r_bullet = addRobotFromURDF(r_bullet,strcat(fullfile(getDrakePath,'systems','plants','test'),'/ground_plane.urdf'),[xstar(1);xstar(2);0],zeros(3,1),struct('floating',false));
  r_bullet = TimeSteppingRigidBodyManipulator(r_bullet,options.dt,options);
  r_bullet = removeCollisionGroupsExcept(r_bullet,{'heel','toe'});
  r_bullet = compile(r_bullet);
end

v = r.constructVisualizer;
v.display_dt = 0.05;

nq = getNumDOF(r);

x0 = xstar;
q0 = x0(1:nq);

% create footstep and ZMP trajectories
step_options.max_num_steps = 100;
step_options.min_num_steps = 2;
step_options.step_height = 0.0;
step_options.step_speed = 0.75;
step_options.follow_spline = true;
step_options.right_foot_lead = true;
step_options.ignore_terrain = false;
step_options.nom_step_width = r.nom_step_width;
step_options.nom_forward_step = r.nom_forward_step;
step_options.max_forward_step = r.max_forward_step;
step_options.goal_type = drc.walking_goal_t.GOAL_TYPE_CENTER;
step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;

footsteps = r.createInitialSteps(x0, navgoal, step_options);
[support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(r, x0, footsteps,step_options);
link_constraints = buildLinkConstraints(r, q0, foottraj);
 
if use_bullet
  for i=1:length(supports)
    supports{i}=supports{i}.setContactSurfaces(-ones(length(supports{i}.bodies),1));
  end
end

ts = 0:0.1:zmptraj.tspan(end);
T = ts(end);

% compute s1,s2 derivatives for controller Vdot computation
s1dot = fnder(V.s1,1);
s2dot = fnder(V.s2,1);

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
  's1dot',s1dot,...
  's2dot',s2dot,...
  'x0',[zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',comtraj,...
  'link_constraints',link_constraints, ...
  'support_times',support_times,...
  'supports',[supports{:}],...
  't_offset',0,...
  'mu',1,...
  'ignore_terrain',false,...
  'y0',zmptraj));

% instantiate QP controller
options.dt = 0.004;
options.slack_limit = 30.0;
options.w = 0.001;
options.lcm_foot_contacts = false;
options.debug = false;

if use_bullet
  options.multi_robot = r_bullet;
end
qp = QPControlBlock(r,ctrl_data,options);

% cascade footstep plan shift block
fs = FootstepPlanShiftBlock(r,ctrl_data,options);
sys = cascade(r,fs);

% feedback QP controller with atlas
ins(1).system = 1;
ins(1).input = 1;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoFeedback(qp,sys,[],[],ins,outs);
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
  opt=traj.eval(ts(i));
  q=opt(1:getNumDOF(r)); 
  com(:,i)=getCOM(r,q);
  comdes(:,i)=comtraj.eval(ts(i));
  zmpdes(:,i)=zmptraj.eval(ts(i));
  
  err = err + sum(abs(comtraj.eval(ts(i)) - com(1:2,i)));
end

if plot_comtraj
  figure(2); 
  clf; 
  subplot(3,1,1);
  plot(ts,zmpdes(1,:),'b');
  hold on;  
  plot(ts,comdes(1,:),'g');
  plot(ts,com(1,:),'r');
  hold off;  
  
  subplot(3,1,2);
  plot(ts,zmpdes(2,:),'b');
  hold on;  
  plot(ts,comdes(2,:),'g');
  plot(ts,com(2,:),'r');
  hold off;  

  subplot(3,1,3); hold on;
  plot(zmpdes(1,:),zmpdes(2,:),'b');
  hold on;  
  plot(comdes(1,:),comdes(2,:),'g');
  plot(com(1,:),com(2,:),'r');
  hold off;  
end


err
if err > length(footsteps)*0.5
  error('drakeWalking unit test failed: error is too large');
  navgoal
end

end
