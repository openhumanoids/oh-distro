function runPinnedReachPlanningLCMBase

%NOTEST
%mode = 1; % 0 = robot, 1 = base
%if mode ==1
%  lcm_url = 'udpm://239.255.12.68:1268?ttl=1';
%else
%  lcm_url = 'udpm://239.255.76.67:7667?ttl=1';
%end
%lcm.lcm.LCM.getSingletonTemp(lcm_url); % only works on mfallons machine

options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% NOTE: JointCommandCoder does not work with model_minimal_contact_with_hands.urdf
%r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_with_hands.urdf',options);

% set initial state to fixed point
%load('../data/atlas_fp3.mat');
% load('../../drake/examples/Atlas/data/atlas_fp3.mat');
% xstar(3) = xstar(3)-0.002;
% r = r.setInitialState(xstar);



% atlas state subscriber
state_frame = r.getStateFrame();
%state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
state_frame.subscribe('EST_ROBOT_STATE');

% end effector subscribers
r_ee = EndEffector(r,'atlas','right_palm',[0;0;0],'RIGHT_PALM_GOAL');
r_ee.frame.subscribe('RIGHT_PALM_GOAL');
l_ee = EndEffector(r,'atlas','left_palm',[0;0;0],'LEFT_PALM_GOAL');
l_ee.frame.subscribe('LEFT_PALM_GOAL');

% TEMP: for now listen for an ee_goal_t message over the EE_PLAN_START
% channel to commence planning
% ee_start_frame = LCMCoordinateFrameWCoder('ee_plan_start',7,'x',JLCMCoder(EndEffectorGoalCoder('atlas','ee_plan_start')));
% ee_start_frame.subscribe('EE_PLAN_START');


joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
committed_plan_listener = RobotPlanListener('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
rejected_plan_listener = RobotPlanListener('atlas',joint_names,true,'REJECTED_ROBOT_PLAN');

x0 = getInitialState(r); 
q0 = x0(1:getNumDOF(r));

kinsol = doKinematics(r,q0);
% r_hand_body = findLink(r,'right_palm');
% l_hand_body = findLink(r,'left_palm');
r_hand_body = findLink(r,'r_hand');
l_hand_body = findLink(r,'l_hand');
rep_goal = [1;forwardKin(r,kinsol,r_hand_body,[0;0;0],true)];
lep_goal = [1;forwardKin(r,kinsol,l_hand_body,[0;0;0],true)];

% Goals are presented in palm frame, must be transformed to hand frame 
% for reach control.
% Using notation similar to KDL.
T_hand_palm_l.xyz = [0 0.1 0];
T_hand_palm_l.M = angle2dcm(1.57079,-0,1.57079, 'ZYX');
T_hand_palm_r.xyz = [0 -0.1 0];
T_hand_palm_r.M = angle2dcm(-1.57079,-0,-1.57079, 'ZYX');

% the inverse transformation that we are interested in.
T_palm_hand_l.xyz = -T_hand_palm_l.xyz;
T_palm_hand_l.M = inv(T_hand_palm_l.M);
T_palm_hand_r.xyz = -T_hand_palm_r.xyz;
T_palm_hand_r.M = inv(T_hand_palm_r.M);


% logic variables
r_goal_received = false; % set when r ee goal is received. Cleared if plan is committed or terminated
l_goal_received = false; % set when l ee goal is received
candidate_plan_published = false;

% get initial state and end effector goals
disp('Listening for goals...');

waiting = true;
while(1)
  rep = getNextMessage(r_ee.frame,1); 
  if (~isempty(rep))
    disp('Right hand goal received.');
    rep_goal = rep;
    r_goal_received = true;
  end
  
  lep = getNextMessage(l_ee.frame,1);
  if (~isempty(lep))
    disp('Left hand goal received.');
    lep_goal = lep;
    l_goal_received = true;
  end
  x = getNextMessage(state_frame,1000);
  if (~isempty(x))
    % disp('Robot state received.');
    % note: setting the desired to actual at 
    % the start of the plan might cause an impulse from gravity sag
    x0 = x;
  end

  if((~isempty(rep))|| (~isempty(lep)))
    if(r_goal_received && l_goal_received)
       disp('Publishing candidate robot plan for left and right end effectors.');
       generateAndPublishManipulationPlan(r,plan_pub,x0,rep_goal,lep_goal); % publish bihanded plan
       candidate_plan_published = true;
    elseif (r_goal_received &&(candidate_plan_published==false))
     disp('Publishing candidate robot plan for right end effector.');
       generateAndPublishManipulationPlan(r,plan_pub,x0,rep_goal,[]);
       candidate_plan_published = true;
    elseif (l_goal_received &&(candidate_plan_published==false))
       disp('Publishing candidate robot plan for left end effector.');
       generateAndPublishManipulationPlan(r,plan_pub,x0,[],lep_goal);
       candidate_plan_published = true; 
    end
  end

%listen to  committed robot plan or rejected robot plan
% channels and clear flags on plan termination.    
  p = committed_plan_listener.getNextMessage(1);
  if (~isempty(p))
    disp('candidate manipulation plan was committed');
       l_goal_received = false;
       r_goal_received = false;
       candidate_plan_published = false;        
  end
  
  p = rejected_plan_listener.getNextMessage(1);
  if (~isempty(p))
    disp('candidate manipulation plan was rejected');
    l_goal_received = false;
     r_goal_received = false;
     candidate_plan_published = false;        
  end

%   start_plan = getNextMessage(ee_start_frame,1);
%   if (~isempty(start_plan))
%     waiting = false;
%   end
end


end
