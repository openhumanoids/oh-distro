function runUnpinnedReachPlanningLCMBase
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


manip_planner = ManipulationPlanner(r);

% atlas state subscriber
state_frame = r.getStateFrame();
%state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
state_frame.subscribe('EST_ROBOT_STATE');

% individual end effector goal subscribers
rh_ee = EndEffector(r,'atlas','right_palm',[0;0;0],'RIGHT_PALM_GOAL');
rh_ee.frame.subscribe('RIGHT_PALM_GOAL');
lh_ee = EndEffector(r,'atlas','left_palm',[0;0;0],'LEFT_PALM_GOAL');
lh_ee.frame.subscribe('LEFT_PALM_GOAL');
rfoot = r.findLink('r_foot');
lfoot = r.findLink('l_foot');
rfoot_pts = rfoot.getContactPoints();
lfoot_pts = lfoot.getContactPoints();
rf_ee = EndEffector(r,'atlas','r_foot',rfoot_pts,'R_FOOT_GOAL');
rf_ee.frame.subscribe('R_FOOT_GOAL');
lf_ee = EndEffector(r,'atlas','l_foot',lfoot_pts,'L_FOOT_GOAL');
lf_ee.frame.subscribe('L_FOOT_GOAL');
h_ee = EndEffector(r,'atlas','head',[0;0;0],'HEAD_GOAL');
h_ee.frame.subscribe('HEAD_GOAL');
h_ee_clear = EndEffector(r,'atlas','head',[0;0;0],'HEAD_GOAL_CLEAR');
h_ee_clear.frame.subscribe('HEAD_GOAL_CLEAR');
% TODO: Incorporate constraints from these channels into the planner.
h_ee_orientation = EndEffector(r,'atlas','head',[0;0;0],'HEAD_ORIENTATION_GOAL');
h_ee_orientation.frame.subscribe('HEAD_ORIENTATION_GOAL');

posture_goal_listener = PresetPostureGoalListener('PRESET_POSTURE_GOAL');

% individual end effector subscribers
rh_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_RIGHT_PALM_MOTION');
lh_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_LEFT_PALM_MOTION');
rf_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_R_FOOT_MOTION');
lf_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_L_FOOT_MOTION');

% constraints for iterative adjustment of plans 
constraint_listener = TrajOptConstraintListener('MANIP_PLAN_CONSTRAINT');

% The following support multiple ee's at the same time
trajoptconstraint_listener = TrajOptConstraintListener('DESIRED_MANIP_PLAN_EE_LOCI');
indexed_trajoptconstraint_listener = AffIndexedTrajOptConstraintListener('DESIRED_MANIP_MAP_EE_LOCI');

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
% r_hand_body = findLink(r,'r_hand');
% l_hand_body = findLink(r,'l_hand');
% rh_ee_goal = forwardKin(r,kinsol,r_hand_body,[0;0;0],1);
% lh_ee_goal = forwardKin(r,kinsol,l_hand_body,[0;0;0],1);


rh_ee_goal = [];
lh_ee_goal = [];
rf_ee_goal = [];
lf_ee_goal = [];
h_ee_goal = [];
lh_ee_constraint = [];
rh_ee_constraint = [];
lf_ee_constraint = [];
rf_ee_constraint = [];
h_ee_constraint = [];

% get initial state and end effector goals
disp('Listening for goals...');
send_status(3,0,0,'Manipulation Planner: Listening for goals...');
while(1)
  rep = getNextMessage(rh_ee.frame,0); 
  if (~isempty(rep))
    disp('Right hand goal received.');
    p=rep(2:4);   rpy=rep(5:7);
%    q=rep(5:8);rpy = quat2rpy(q);
    rh_ee_goal=[p(:);rpy(:)];
  end
  
  lep = getNextMessage(lh_ee.frame,0);
  if (~isempty(lep))
    disp('Left hand goal received.');
    p=lep(2:4);   rpy=lep(5:7);
    %    q=lep(5:8);rpy = quat2rpy(q);
    lh_ee_goal=[p(:);rpy(:)];
  end
  
  rfep = getNextMessage(rf_ee.frame,0); 
  if (~isempty(rfep))
    disp('Right foot goal received.');
    p=rfep(2:4);   rpy=rfep(5:7);
%    q=rep(5:8);rpy = quat2rpy(q);
    rf_ee_goal=[p(:);rpy(:)];
  end
  
  lfep = getNextMessage(lf_ee.frame,0);
  if (~isempty(lfep))
    disp('Left foot goal received.');
    p=lfep(2:4);   rpy=lfep(5:7);
    %    q=lep(5:8);rpy = quat2rpy(q);
    lf_ee_goal=[p(:);rpy(:)];
  end
  
  hep = getNextMessage(h_ee.frame,0);
  if (~isempty(hep))
    disp('head goal received.');
    p = hep(2:4);   
    rpy = hep(5:7);
    %    q=lep(5:8);rpy = quat2rpy(q);
    h_ee_goal = [p(:); rpy(:)];
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
       lh_ee_constraint = x;
     elseif((num_links==1)&&(strcmp(x.name,'right_palm')))
       disp('received keyframe constraint for right hand');
       rh_ee_constraint = x;
     elseif((num_links==1)&&(strcmp(x.name,'l_foot')))
       disp('received keyframe constraint for left foot');
       lf_ee_constraint = x;
       x_pos = x.desired_pose;
       rpy = quat2rpy(x_pos(4:7));
       trans_mat = HT(x_pos(1:3),rpy(1),rpy(2),rpy(3));
       lf_ee_pos = trans_mat*[lfoot_pts;ones(1,size(lfoot_pts,2))];
       lf_ee_constraint.desired_pose = [lf_ee_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),x_pos(4:7))];
     elseif((num_links==1)&&(strcmp(x.name,'r_foot')))
       disp('received keyframe constraint for right foot');
       rf_ee_constraint = x;
       x_pos = x.desired_pose;
       rpy = quat2rpy(x_pos(4:7));
       trans_mat = HT(x_pos(1:3),rpy(1),rpy(2),rpy(3));
       rf_ee_pos = trans_mat*[rfoot_pts;ones(1,size(rfoot_pts,2))];
       rf_ee_constraint.desired_pose = [rf_ee_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),x_pos(4:7))];
     elseif((num_links==1)&&(strcmp(x.name,'head')))
       disp('received keyframe constraint for head');
       h_ee_constraint = x;
     else
      disp('Manip planner currently expects one constraint at a time') ; 
     end     
     manip_planner.adjustAndPublishManipulationPlan(x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint);
       
  end
  
  lh_ee_traj= lh_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(lh_ee_traj))
      disp('Left hand traj goal received.');
      p = lh_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = lh_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      lep = [p(:);q(:)];
      rpy = quat2rpy(q);
      lh_ee_goal=[p(:);rpy(:)];
  end
  
  rh_ee_traj= rh_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(rh_ee_traj))
      disp('Right hand traj goal received.');
      p = rh_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = rh_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      rep = [p(:);q(:)];
      rpy = quat2rpy(q);
      rh_ee_goal=[p(:);rpy(:)];
  end
  
  lf_ee_traj= lf_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(lf_ee_traj))
      disp('Left foot traj goal received.');
      p = lf_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = lf_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      rpy = quat2rpy(q);
	  trans_mat = HT(p,rpy(1),rpy(2),rpy(3));
	  lep_pos = trans_mat*[lfoot_pts,ones(1,size(lfoot_pts,2))];
      lep = [lep_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),q(:))];
      lf_ee_goal=[lep_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),rpy(1:3))];
  end

  rf_ee_traj= rf_ee_motion_command_listener.getNextMessage(0);
  if(~isempty(rf_ee_traj))
      disp('Right hand traj goal received.');
      p = rf_ee_traj(end).desired_pose(1:3);% for now just take the end state
      q = rf_ee_traj(end).desired_pose(4:7);q=q/norm(q);
      rpy = quat2rpy(q);
	  trans_mat = HT(p,rpy(1),rpy(2),rpy(3));
	  rep_pos = trans_mat*[rfoot_pts,ones(1,size(rfoot_pts,2))];
      rep = [rep_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),q(:))];
      rf_ee_goal=[rep_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),rpy(1:3))];
  end
  
  if( (~isempty(rep))|| (~isempty(lep)) ||(~isempty(rfep))||...
      (~isempty(lfep))||(~isempty(rh_ee_traj))||(~isempty(lh_ee_traj))||...
      (~isempty(rf_ee_traj))||(~isempty(lf_ee_traj)))
      manip_planner.generateAndPublishManipulationPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal); 
  end
  
  trajoptconstraint= trajoptconstraint_listener.getNextMessage(0);
  if(~isempty(trajoptconstraint))
      disp('time indexed traj opt constraint for manip plan received .');
      
      % cache the a subset of fields as indices for aff_indexed_plans
      timestamps =[trajoptconstraint.time];
      ee_names = {trajoptconstraint.name};
      ee_loci = zeros(6,length(ee_names));
      for i=1:length(ee_names),
          p = trajoptconstraint(i).desired_pose(1:3);% for now just take the end state
          q = trajoptconstraint(i).desired_pose(4:7);q=q/norm(q);
          rpy = quat2rpy(q);
          ee_loci(:,i)=[p(:);rpy(:)];
      end
      manip_planner.generateAndPublishManipulationPlan(x0,ee_names,ee_loci,timestamps);
  end    
  
  
  indexed_trajoptconstraint= indexed_trajoptconstraint_listener.getNextMessage(0);  
  if(~isempty(indexed_trajoptconstraint))
      disp('Aff indexed traj opt constraint for manip map received .');
       
      % cache the a subset of fields as indices for aff_indexed_plans 
      affIndices =indexed_trajoptconstraint;
      affIndices = rmfield(affIndices,{'name';'desired_pose'});
      
      ee_names = {indexed_trajoptconstraint.name};
      ee_loci = zeros(6,length(ee_names));
      for i=1:length(ee_names),
          p = indexed_trajoptconstraint(i).desired_pose(1:3);% for now just take the end state
          q = indexed_trajoptconstraint(i).desired_pose(4:7);q=q/norm(q);
          rpy = quat2rpy(q);
          ee_loci(:,i)=[p(:);rpy(:)];
      end      
      manip_planner.generateAndPublishManipulationMap(x0,ee_names,ee_loci,affIndices);
  end
  

  posture_goal =posture_goal_listener.getNextMessage(0);  
  if(~isempty(posture_goal))
      disp('Preset Posture goal received .');
      if(posture_goal.preset==1)
       d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));%standing hands down
      elseif(posture_goal.preset==2)
       d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_hands_up.mat'));%standing hands up
      elseif(posture_goal.preset==3)
        d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));%seated hands down
      elseif(posture_goal.preset==4) 
        d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'));%seated hands up   
      end
      q_desired = d.xstar(1:getNumDOF(r));
      q_desired(1:6) = x0(1:6); % fix pelvis pose to current
      manip_planner.generateAndPublishPosturePlan(x0,q_desired);
  end

%listen to  committed robot plan or rejected robot plan
% channels and clear flags on plan termination.    
  p = committed_plan_listener.getNextMessage(0);
  if (~isempty(p))
    disp('candidate manipulation plan was committed');
       rh_ee_goal = [];
       lh_ee_goal = [];
       rf_ee_goal = [];
       lf_ee_goal = [];       
       lh_ee_constraint = [];
       rh_ee_constraint = [];
       lf_ee_constraint = [];
       rf_ee_constraint = [];
       %h_ee_goal = [];
       h_ee_constraint = [];
  end
  
  p = rejected_plan_listener.getNextMessage(0);
  if (~isempty(p))
    disp('candidate manipulation plan was rejected');
    rh_ee_goal = [];
    lh_ee_goal = [];
    rf_ee_goal = [];
    lf_ee_goal = [];
    lh_ee_constraint = [];
    rh_ee_constraint = [];
    lf_ee_constraint = [];
    rf_ee_constraint = [];
    %h_ee_goal = [];
    h_ee_constraint = [];
  end
  
  p = getNextMessage (h_ee_clear.frame, 0);
  if (~isempty(p))
      disp ('Clearing head goal pose');
      h_ee_goal = [];
  end

end


end
