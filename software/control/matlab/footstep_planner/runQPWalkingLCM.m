function runQPWalkingLCM(lcm_plan, goal_x, goal_y, goal_yaw)

if nargin < 4; goal_yaw = 0.0; end
if nargin < 3; goal_y = 1.0; end
if nargin < 2; goal_x = 0.0; end
if nargin < 1; lcm_plan = true; end

options.floating = true;
options.dt = 0.002;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
d = load('../data/atlas_fp.mat');
xstar = d.xstar;
r = r.setInitialState(xstar);
% set initial conditions in gazebo
state_frame = getStateFrame(r);
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');

nq = getNumDOF(r);
nu = getNumInputs(r);
x0 = xstar;
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

pose = [goal_x;goal_y;0;0;0;goal_yaw];

if ~lcm_plan
  [rfoot, lfoot] = planFootsteps(r, x0, pose, struct('plotting', true, 'interactive', false));
else
%   footstep_plan_listener = FootstepPlanListener('atlas', 'COMMITTED_FOOTSTEP_PLAN');
  footstep_plan_listener = FootstepPlanListener('atlas', 'CANDIDATE_FOOTSTEP_PLAN');

  disp('Listening for footstep plans...');
  waiting = true;
  foottraj = [];
  while waiting
    foottraj = footstep_plan_listener.getNextMessage(100);
    if (~isempty(foottraj))
      disp('footstep plan received.');
      waiting = false;
    end
  end
  
  rfoot = foottraj(3:end,find(foottraj(1,:)==1));
  rfoot(2,:) = -rfoot(2,:);
  lfoot = foottraj(3:end,find(foottraj(1,:)==0));
  lfoot(2,:) = -lfoot(2,:);
end

[zmptraj,foottraj,~,~,supptraj] = planZMPandHeelToeTrajectory(r, q0, rfoot, lfoot, 1.0);
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

disp('Computing robot plan...');
v = r.constructVisualizer;
v.display_dt = 0.05;
htraj = [];
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],foottraj.right.orig.eval(t),lfoot_body,[0;0;0],foottraj.left.orig.eval(t),options);
  else
    q = q0;
  end
  com = getCOM(r,q(:,i));
  htraj = [htraj com(3)];
  v.draw(t,q(:,i));
end
htraj = PPTrajectory(spline(ts,htraj));

% publish robot plan
disp('Publishing robot plan...');
xtraj = zeros(getNumStates(r),length(ts));
xtraj(1:getNumDOF(r),:) = q;
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
plan_pub.publish(ts,xtraj);

% figure(2); 
% clf; 
% subplot(3,1,1); hold on;
% fnplt(zmptraj(1));
% fnplt(comtraj(1));
% subplot(3,1,2); hold on;
% fnplt(zmptraj(2));
% fnplt(comtraj(2));
% subplot(3,1,3); hold on;
% fnplt(htraj);

disp('Computing ZMP controller...');
limp = LinearInvertedPendulum(htraj);
[c, V] = ZMPtracker(limp,zmptraj);
zmpdata = SharedDataHandle(struct('V',V,'h',com(3),'c',c));

% instantiate QP controller
options.slack_limit = 100.0;
options.w = 1.0;
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

% walking foot support trajectory
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 3;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supptraj,qp,[],ins,outs);
clear ins outs;

% cascade PD outputs based on qdes trajectory
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(pd,sys,[],ins,outs);
clear ins outs;

% disp('Waiting for robot plan confirmation...');
% plan_listener = RobotPlanListener('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
% waiting = true;
% while waiting
%   rplan = plan_listener.getNextMessage(100);
%   if (~isempty(rplan))
%     % for now don't do anything with it, just use it as a flag
%     disp('Plan confirmed. Executing...');
%     waiting = false;
%   end
% end

state_frame.subscribe('TRUE_ROBOT_STATE');
input_frame = getInputFrame(r);
t_offset = -1;
t= -1;
traj = [];
ts = [];
disp('waiting...');
while true
  [x,tsim] = getNextMessage(state_frame,1);
  if (~isempty(x))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    ts = [ts t];
    traj = [traj x];
    u = sys.output(t,[],[x;x]);
    input_frame.publish(t,u,'JOINT_COMMANDS');
  end
end
% 
% for i=1:length(ts)
%   x=traj(:,i);
%   q=x(1:getNumDOF(r)); 
%   com(:,i)=getCOM(r,q);
% end

% figure(2);
% subplot(3,1,1);
% plot(ts,com(1,:),'r');
% subplot(3,1,2);
% plot(ts,com(2,:),'r');
% subplot(3,1,3);
% plot(ts,com(3,:),'r');

 
% options.timekeeper = 'drake/lcmTimeKeeper'; 
% runLCM(sys,[],options);

end
