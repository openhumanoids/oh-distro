function runReachPlanningLCMBase
%NOTEST
mode = 1; % 0 = robot, 1 = base
if mode ==1
  lcm_url = 'udpm://239.255.12.68:1268?ttl=1';
else
  lcm_url = 'udpm://239.255.76.67:7667?ttl=1';
end
lcm.lcm.LCM.getSingletonTemp(lcm_url); % only works on mfallons machine

options.floating = true;
options.dt = 0.001;
r = Atlas('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp3.mat');
xstar(3) = xstar(3)-0.002;
r = r.setInitialState(xstar);

% atlas state subscriber
state_frame = r.getStateFrame();
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
state_frame.subscribe('EST_ROBOT_STATE');

% end effector subscribers
r_ee = EndEffector(r,'atlas','r_hand',[0;0;0],'R_HAND_GOAL');
r_ee.frame.subscribe('R_HAND_GOAL');
l_ee = EndEffector(r,'atlas','l_hand',[0;0;0],'L_HAND_GOAL');
l_ee.frame.subscribe('L_HAND_GOAL');

% TEMP: for now listen for an ee_goal_t message over the EE_PLAN_START
% channel to commence planning
ee_start_frame = LCMCoordinateFrameWCoder('ee_plan_start',7,'x',JLCMCoder(EndEffectorGoalCoder('atlas','ee_plan_start')));
ee_start_frame.subscribe('EE_PLAN_START');

x0 = getInitialState(r); 
q0 = x0(1:getNumDOF(r));

kinsol = doKinematics(r,q0);
r_hand_body = findLink(r,'r_hand');
l_hand_body = findLink(r,'l_hand');
rep_goal = [1;forwardKin(r,kinsol,r_hand_body,[0;0;0],true)];
lep_goal = [1;forwardKin(r,kinsol,l_hand_body,[0;0;0],true)];

% get initial state and end effector goals
disp('Listening for goals...');
waiting = true;
while waiting
  rep = getNextMessage(r_ee.frame,1);
  if (~isempty(rep))
    disp('Right hand goal received.');
    rep_goal = rep;
  end
  lep = getNextMessage(l_ee.frame,1);
  if (~isempty(lep))
    disp('Left hand goal received.');
    lep_goal = lep;
  end
  x = getNextMessage(state_frame,1);
  if (~isempty(x))
    % note: setting the desired to actual at 
    % the start of the plan might cause an impulse from gravity sag
    %x0 = x;
  end
  start_plan = getNextMessage(ee_start_frame,1);
  if (~isempty(start_plan))
    waiting = false;
  end
end

disp('Generating plan...');

% generate robot plan
T = 5.0; % seconds, hard coded for now
dt = 0.1;
ts = 0:dt:T; % plan timesteps

q0 = x0(1:getNumDOF(r));

% get foot positions
kinsol = doKinematics(r,q0);
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

% compute fixed COM goal
gc = contactPositions(r,q0);
k = convhull(gc(1:2,:)');
com0 = getCOM(r,q0);
comgoal = [mean(gc(1:2,k),2);com0(3)];

% compute EE trajectories

rhand0 = forwardKin(r,kinsol,r_hand_body,[0;0;0],true);
lhand0 = forwardKin(r,kinsol,l_hand_body,[0;0;0],true);

rpalmT = rep_goal(2:7); % skip active/inactive bit
lpalmT = lep_goal(2:7);

% transform to hand coordinate frame
% R_ptoh = angle2dcm(0, 3.1416, 1.57079, 'XYZ'); % transform from atlas urdf
% Tr = zeros(4);
% Tr(1:3,1:3) = R_ptoh;
% Tr(1:4,4) = [0; -0.1; -0.03; 1];
% Tl = zeros(4);
% Tl(1:3,1:3) = R_ptoh;
% Tl(1:4,4) = [0; 0.1; -0.03; 1];
% 
% rhandT = zeros(6,1);
% lhandT = zeros(6,1);
% rhandT(1:3) = [eye(3),zeros(3,1)]*Tr*[rpalmT(1:3);1];
% lhandT(1:3) = [eye(3),zeros(3,1)]*Tl*[lpalmT(1:3);1];
% rhandT(4:6) = dcm2angle(R_ptoh*angle2dcm(rpalmT(4),rpalmT(5),rpalmT(6),'XYZ'),'XYZ');
% lhandT(4:6) = dcm2angle(R_ptoh*angle2dcm(lpalmT(4),lpalmT(5),lpalmT(6),'XYZ'),'XYZ');

rhandT = rpalmT;
lhandT = lpalmT;

% % ignore pose
rhand0 = rhand0(1:3);
lhand0 = lhand0(1:3);
rhandT = rhandT(1:3);
lhandT = lhandT(1:3);

r_hand_pos = PPTrajectory(foh([0,T],[rhand0,rhandT]));
l_hand_pos = PPTrajectory(foh([0,T],[lhand0,lhandT]));

ind = getActuatedJoints(r);
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 100;
cost.pelvis_y = 100;
cost.pelvis_z = 100;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
ikoptions = struct();
ikoptions.Q = diag(cost(1:getNumDOF(r)));
ikoptions.q_nom = q0;

v = r.constructVisualizer();

q = q0;
q_d = q(ind);
for i=2:length(ts)
  t = ts(i);
  q(:,i) = inverseKin(r,q(:,i-1),0,comgoal,rfoot_body,rfoot0, ...
      lfoot_body,lfoot0,r_hand_body,r_hand_pos.eval(t), ...
      l_hand_body,l_hand_pos.eval(t),ikoptions);
  q_d(:,i) = q(ind,i);
  v.draw(t,q(:,i));
end
qd_frame = AtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);

% publish robot plan
disp('Publishing plan...');
xtraj = zeros(getNumStates(r),length(ts));
xtraj(1:getNumDOF(r),:) = q;
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
plan_pub.publish(ts,xtraj,des_traj);

end