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


manip_planner = ManipulationPlanner(r);
constraint_listener = TrajOptConstraintListener('MANIP_PLAN_CONSTRAINT');
r_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_RIGHT_PALM_MOTION');
l_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_LEFT_PALM_MOTION');

joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
% committed_plan_listener = RobotPlanListener('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
% rejected_plan_listener = RobotPlanListener('atlas',joint_names,true,'REJECTED_ROBOT_PLAN');
committed_plan_listener = RobotPlanListener('COMMITTED_ROBOT_PLAN',true);
rejected_plan_listener = RobotPlanListener('REJECTED_ROBOT_PLAN',true);

x0 = getInitialState(r); 
q0 = x0(1:getNumDOF(r));

kinsol = doKinematics(r,q0);
% r_hand_body = findLink(r,'right_palm');
% l_hand_body = findLink(r,'left_palm');
r_hand_body = findLink(r,'r_hand');
l_hand_body = findLink(r,'l_hand');
r_ee_goal = forwardKin(r,kinsol,r_hand_body,[0;0;0],1);
l_ee_goal = forwardKin(r,kinsol,l_hand_body,[0;0;0],1);
r_ee_constraint = [];
l_ee_constraint = [];
% logic variables
r_goal_received = false; % set when r ee goal is received. Cleared if plan is committed or terminated
l_goal_received = false; % set when l ee goal is received
l_constraint_received = false;
r_constraint_received = false;

% get initial state and end effector goals
disp('Listening for goals...');

waiting = true;
while(1)
  rep = getNextMessage(r_ee.frame,0); 
  if (~isempty(rep))
    disp('Right hand goal received.');
    p=rep(2:4);   rpy=rep(5:7);
%    q=rep(5:8);rpy = quat2rpy(q);
    r_ee_goal=[p(:);rpy(:)];
    r_goal_received = true;
  end
  
  lep = getNextMessage(l_ee.frame,0);
  if (~isempty(lep))
    disp('Left hand goal received.');
    p=lep(2:4);   rpy=lep(5:7);
    %    q=lep(5:8);rpy = quat2rpy(q);
    l_ee_goal=[p(:);rpy(:)];
    l_goal_received = true;
  end
  
  [x,ts] = getNextMessage(state_frame,0);
  if (~isempty(x))
    %  fprintf('received state at time %f\n',ts);
    % disp('Robot state received.');
    % note: setting the desired to actual at 
    % the start of the plan might cause an impulse from gravity sag
    x0 = x;
  end
  
  x= constraint_listener.getNextMessage(0); % not a frame
  if(~isempty(x))
     num_links = length(x.time);
     if((num_links==1)&&(strcmp(x.name,'left_palm')))
       disp('received keyframe constraint for left hand'); 
       l_constraint_received = true;
       l_ee_constraint = x;
     elseif((num_links==1)&&(strcmp(x.name,'right_palm')))
       disp('received keyframe constraint for right hand');
       r_constraint_received = true;
       r_ee_constraint = x;
     else
      disp('Manip planner currently expects one constraint at a time') ; 
     end     
     
    if(r_constraint_received && l_constraint_received)
       disp('adjusting candidate robot plan given left and right end effector keyframe constraints.');
       manip_planner.adjustAndPublishManipulationPlan(x0,r_ee_constraint,l_ee_constraint); 
    elseif (r_constraint_received)
       disp('adjusting candidate robot plan given right end effector keyframe constraint.');
       manip_planner.adjustAndPublishManipulationPlan(x0,r_ee_constraint,[]);
    elseif (l_constraint_received)
       disp('adjusting candidate robot plan given left end effector keyframe constraint..');
       manip_planner.adjustAndPublishManipulationPlan(x0,[],l_ee_constraint);
    end    
     
  end
  
  
  l_ee_traj= l_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(l_ee_traj))
      disp('Left hand traj goal received.');
      p = l_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = l_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      lep = [p(:);q(:)];
      rpy = quat2rpy(q);
      l_ee_goal=[p(:);rpy(:)];
      l_goal_received = true;
  end
  
  r_ee_traj= r_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(r_ee_traj))
      disp('Right hand traj goal received.');
      p = r_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = r_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      rep = [p(:);q(:)];
      rpy = quat2rpy(q);
      r_ee_goal=[p(:);rpy(:)];
      r_goal_received = true;
  end
  

  if((~isempty(rep))|| (~isempty(lep)))
    if(r_goal_received && l_goal_received)
       disp('Publishing candidate robot plan for left and right end effectors.');
       manip_planner.generateAndPublishManipulationPlan(x0,r_ee_goal,l_ee_goal); 
    elseif (r_goal_received)
       disp('Publishing candidate robot plan for right end effector.');
       manip_planner.generateAndPublishManipulationPlan(x0,r_ee_goal,[]);
    elseif (l_goal_received)
       disp('Publishing candidate robot plan for left end effector.');
       manip_planner.generateAndPublishManipulationPlan(x0,[],l_ee_goal);
    end
  end

%listen to  committed robot plan or rejected robot plan
% channels and clear flags on plan termination.    
  p = committed_plan_listener.getNextMessage(0);
  if (~isempty(p))
    disp('candidate manipulation plan was committed');
       l_goal_received = false;
       r_goal_received = false;
       l_constraint_received = false;
       r_constraint_received = false;
  end
  
  p = rejected_plan_listener.getNextMessage(0);
  if (~isempty(p))
    disp('candidate manipulation plan was rejected');
    l_goal_received = false;
    r_goal_received = false;
    l_constraint_received = false;
    r_constraint_received = false;
  end

end


end
