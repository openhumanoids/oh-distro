function runKeyFramePlannersLCMBase(varargin)
% Usage:
% ===============================================================
% runKeyFramePlannersLCMBase(hardware_mode)
% or
% runKeyFramePlannersLCMBase(hardware_mode) assumes sandia hands by default
% @param hardware_mode   -- -1 for sim mode
%                           -2 BDI_Manip_Mode(upper body only)
%                           -3 for BDI_User

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

switch nargin
    case 1
        hardware_mode = varargin{1}; % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
        l_hand_mode = 1;
        r_hand_mode = 1;
    otherwise
        error('Incorrect usage of runKeyFramePlannersLCMBase. Undefined number of varargin. (hardware_mode)')
end

% get the robot model first
% @param l_hand_mode          - 0 no left hand
%                            - 1 sandia left hand
%                            - 2 irobot left hand
%                            - 4 robotiq left hand
% @param r_hand_mode         - 0 no right hand
%                            - 1 sandia right hand
%                            - 2 irobot right hand
%                            - (3 seems to be used for HOSE TASK in RobotModelListener.m)
%                            - 4 robotiq right hand
getModelFlag = false;
model_listener = RobotModelListener('ROBOT_MODEL');
while(~getModelFlag)
  data = model_listener.getNextMessage(5);
  if(~isempty(data))
    getModelFlag = true;
    l_hand_mode = data.left_hand_mode;
    r_hand_mode = data.right_hand_mode;
    if(~isempty(strfind(data.robot_name,'Hose')) && r_hand_mode == 2)
      r_hand_mode = 3;
      send_status(4,0,0,'irobot hand for hose task');
    end
    if(l_hand_mode == 0)
      l_hand_str = 'no hand';
    elseif(l_hand_mode == 1)
      l_hand_str = 'sandia hand';
    elseif(l_hand_mode == 2)
      l_hand_str = 'irobot hand';
    elseif(l_hand_mode == 4)
      l_hand_str = 'robotiq hand';      
    end
    if(r_hand_mode == 0)
      r_hand_str = 'no hand';
    elseif(r_hand_mode == 1)
      r_hand_str = 'sandia hand';
    elseif(r_hand_mode == 2 || r_hand_mode == 3)
      r_hand_str = 'irobot hand';
    elseif(r_hand_mode == 4)
      r_hand_str = 'robotiq hand';      
    end
    send_status(4,0,0,sprintf('receive model with left %s, right %s\n',l_hand_str,r_hand_str));
  end
end
options.floating = true;
options.dt = 0.001;
if(l_hand_mode == 0 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RN.urdf'),options);
elseif(l_hand_mode == 0 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RS.urdf'),options);
elseif(l_hand_mode == 0 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RI.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LS_RN.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'),options);
elseif(l_hand_mode == 1 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LS_RI.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RN.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 1)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RS.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 2)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RI.urdf'),options);
elseif(l_hand_mode == 2 && r_hand_mode == 3)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LI_RI_Hose.urdf'),options);
elseif(l_hand_mode == 4 && r_hand_mode == 0)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RN.urdf'),options);
elseif(l_hand_mode == 0 && r_hand_mode == 4)
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RN.urdf'),options);
else
  error('The urdf for the model does not exist');
  robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_LN_RN.urdf'),options);
end
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

% if(r_hand_mode == 1)
% right_hand_links = [robot.findLinkInd('r_hand'),robot.findLinkInd('right_f0_0'),robot.findLinkInd('right_f0_1'),...
%   robot.findLinkInd('right_f0_2'),robot.findLinkInd('right_f1_0'),robot.findLinkInd('right_f1_1'),...
%   robot.findLinkInd('right_f1_2'),robot.findLinkInd('right_f2_0'),robot.findLinkInd('right_f2_1'),...
%   robot.findLinkInd('right_f2_2'),robot.findLinkInd('right_f3_0'),robot.findLinkInd('right_f3_1'),robot.findLinkInd('right_f3_2')];
% robot.collision_filter_groups('right_hand') = CollisionFilterGroup();
% robot = robot.addLinksToCollisionFilterGroup(right_hand_links,'right_hand',1);
% robot = compile(robot);
% robot = robot.addToIgnoredListOfCollisionFilterGroup({'right_hand'},'right_hand');
% robot = compile(robot);
% end
% if(l_hand_mode == 1)
% left_hand_links = [robot.findLinkInd('l_hand'),robot.findLinkInd('left_f0_0'),robot.findLinkInd('left_f0_1'),robot.findLinkInd('left_f0_2'),...
%   robot.findLinkInd('left_f1_0'),robot.findLinkInd('left_f1_1'),robot.findLinkInd('left_f1_2'),...
%   robot.findLinkInd('left_f2_0'),robot.findLinkInd('left_f2_1'),robot.findLinkInd('left_f2_2'),...
%   robot.findLinkInd('left_f3_0'),robot.findLinkInd('left_f3_1'),robot.findLinkInd('left_f3_2')];
% robot.collision_filter_groups('left_hand') = CollisionFilterGroup();
% robot = robot.addLinksToCollisionFilterGroup(left_hand_links,'left_hand',1);
% robot = compile(robot);
% robot = robot.addToIgnoredListOfCollisionFilterGroup({'left_hand'},'left_hand');
% robot = compile(robot);
% end


if(nargin<1)
    hardware_mode = 1;  % 1 for sim mode, 2 BDI_Manip_Mode(upper body only), 3 for BDI_User
end
% l_hand_listener = drc.control.HandStateListener(l_hand_mode,'left','EST_ROBOT_STATE');
% r_hand_listener = drc.control.HandStateListener(r_hand_mode,'right','EST_ROBOT_STATE');
l_hand_frame = handFrame(l_hand_mode,'left');
r_hand_frame = handFrame(r_hand_mode,'right');

reaching_planner = HoseMatingReachingPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % single or multiple/successively specified ee constraints
manip_planner = ManipulationPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % ee motion constraints and point wise IK for manip plans and maps
posture_planner = PosturePlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); %posture and posture preset plans
endpose_planner = HoseMatingEndPosePlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); %search for pose given ee constraints
wholebody_planner = WholeBodyPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode);%given a time ordered set ee constraints, performs a whole body plan
keyframe_adjustment_engine = KeyframeAdjustmentEngine(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % Common keyframe adjustment for all the above planners

% atlas state subscriber
atlas_state_frame = atlas.getStateFrame();
%state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
atlas_state_frame.subscribe('EST_ROBOT_STATE');

% individual end effector goal subscribers
rh_ee = EndEffectorListener('RIGHT_PALM_GOAL');
lh_ee = EndEffectorListener('LEFT_PALM_GOAL');
rfoot = atlas.findLinkInd('r_foot');
lfoot = atlas.findLinkInd('l_foot');
rfoot_pts = getContactPoints(getBody(atlas,rfoot));
lfoot_pts = getContactPoints(getBody(atlas,lfoot));
rf_ee = EndEffectorListener('R_FOOT_GOAL');
lf_ee = EndEffectorListener('L_FOOT_GOAL');
des_arc_speed_listener =DesiredSpeedListener('DESIRED_EE_ARC_SPEED');
des_joint_speed_listener =DesiredSpeedListener('DESIRED_JOINT_SPEED');

h_ee = EndEffectorListener('HEAD_GOAL');
h_ee_clear = EndEffectorListener('HEAD_GOAL_CLEAR');

h_ee_orientation = EndEffectorListener('HEAD_ORIENTATION_GOAL');
lh_ee_orientation = EndEffectorListener('LEFT_PALM_ORIENTATION_GOAL');
rh_ee_orientation = EndEffectorListener('RIGHT_PALM_ORIENTATION_GOAL');


h_ee_gaze = EndEffectorListener('HEAD_GAZE_GOAL');
lidar_ee_gaze = EndEffectorListener('HEAD_WEAK_GAZE_GOAL');
lh_ee_gaze = EndEffectorListener('LEFT_PALM_GAZE_GOAL');
rh_ee_gaze = EndEffectorListener('RIGHT_PALM_GAZE_GOAL');

lh_ee_clear = EndEffectorListener('LEFT_PALM_GOAL_CLEAR');
rh_ee_clear = EndEffectorListener('RIGHT_PALM_GOAL_CLEAR');

preset_posture_goal_listener = PresetPostureGoalListener('PRESET_POSTURE_GOAL');
posture_goal_listener = PostureGoalListener('POSTURE_GOAL');
pose_goal_listener = TrajOptConstraintListener('POSE_GOAL');
manip_plan_mode_listener = ManipPlanModeListener('MANIP_PLANNER_MODE_CONTROL');


% WorkspaceURDF subscriber
% workspace_urdf_listener = WorkspaceURDFListener('COLLISION_AVOIDANCE_URDFS');
% urdf_names = {};
% aff_manager = AffordanceManager(atlas,robot,l_hand_frame,...
%   r_hand_frame,'AFFORDANCE_COLLECTION');
% individual end effector subscribers
rh_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_RIGHT_PALM_MOTION');
lh_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_LEFT_PALM_MOTION');
rf_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_R_FOOT_MOTION');
lf_ee_motion_command_listener = TrajOptConstraintListener('DESIRED_L_FOOT_MOTION');

% constraints for iterative adjustment of plans
constraint_listener = TrajOptConstraintListener('MANIP_PLAN_CONSTRAINT');
plan_pelvis_adjust_listener = PlanAdjustModeListener('ADJUST_PLAN_TO_CURRENT_PELVIS_POSE');
sse_compensation_listener = PlanAdjustModeListener('MOVE_TO_COMPENSATE_SSE');
plan_adjust_and_reach_listener= PlanAdjustModeListener('ADJUST_PLAN_AND_REACH'); 
manip_plan_initseed_toggle_listener= PlanAdjustModeListener('MANIP_PLAN_FROM_CURRENT_STATE'); % Turn off when debugging


% The following support multiple ee's at the same time
trajoptconstraint_listener = TrajOptConstraintListener('DESIRED_MANIP_PLAN_EE_LOCI');
indexed_trajoptconstraint_listener = AffIndexedTrajOptConstraintListener('DESIRED_MANIP_MAP_EE_LOCI');
wholebodytrajoptconstraint_listener = TrajOptConstraintListener('DESIRED_WHOLE_BODY_PLAN_EE_GOAL_SEQUENCE');

joint_names = atlas.getStateFrame.coordinates(1:getNumDOF(atlas));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');
% committed_plan_listener = RobotPlanListener('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
% rejected_plan_listener = RobotPlanListener('atlas',joint_names,true,'REJECTED_ROBOT_PLAN');
committed_plan_listener = RobotPlanListener('COMMITTED_ROBOT_PLAN',true,joint_names);
rejected_plan_listener = RobotPlanListener('REJECTED_ROBOT_PLAN',true,joint_names);
stored_plan_listener = RobotKeyframePlanListener('STORED_ROBOT_PLAN',true,joint_names);



rh_ee_goal = [];
lh_ee_goal = [];
rf_ee_goal = [];
lf_ee_goal = [];
h_ee_goal = [];
lidar_ee_goal = [];

lh_ee_constraint = [];
rh_ee_constraint = [];
lf_ee_constraint = [];
rf_ee_constraint = [];
pelvis_constraint = [];
com_constraint = [];
h_ee_constraint = []; % persists

msg_timeout = 5; % ms

% get initial state and end effector goals
disp('Listening for goals...');
send_status(3,0,0,'KeyframePlanners: Listening for goals...');

ee_goal_type_flags.lh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
ee_goal_type_flags.rh = 0;
ee_goal_type_flags.h  = 0;
ee_goal_type_flags.lf = 0;
ee_goal_type_flags.rf = 0;
ee_goal_type_flags.lidar = 0;
 
while(1)
%   aff_manager.updateWmessage(msg_timeout);
%   % update the aff_data
%   urdf_msg = workspace_urdf_listener.getNextMessage(msg_timeout);
%   if(~isempty(urdf_msg))
%     aff_idx = find(aff_manager.aff_uid == urdf_msg.uid);
%     robot = addAffordance2robot(robot,urdf_msg.urdf_file,aff_manager.aff_xyz(:,aff_idx),...
%       aff_manager.aff_rpy(:,aff_idx),struct('floating',false));
%     
%     aff_manager.updateWcollisionObject(robot.getStateFrame,urdf_msg.uid,atlas_state_frame,...
%       l_hand_frame,r_hand_frame);
%     reaching_planner.updateRobot(robot);
%     manip_planner.updateRobot(robot);
%     posture_planner.updateRobot(robot);
%     endpose_planner.updateRobot(robot);
%     wholebody_planner.updateRobot(robot);
%     keyframe_adjustment_engine.updateRobot(robot);
%     
%     urdf_names = [urdf_names,{urdf_msg.urdf_file}];
%   end
  
    modeset=manip_plan_mode_listener.getNextMessage(msg_timeout);
    if(~isempty(modeset))
        disp('Manip Planner mode control msg received .');
        if(modeset.mode==drc.manip_plan_control_t.IKSEQUENCE_ON)
            reaching_planner.setPlanningMode(1);
            manip_planner.setPlanningMode(1);
            send_status(3,0,0,'KeyframePlanners:IKSEQUENCE_ON MODE');
        elseif(modeset.mode==drc.manip_plan_control_t.IKSEQUENCE_OFF)
            reaching_planner.setPlanningMode(2);
            manip_planner.setPlanningMode(2);
            send_status(3,0,0,'KeyframePlanners:IKSEQUENCE_OFF MODE');
        elseif(modeset.mode==drc.manip_plan_control_t.TELEOP)
            reaching_planner.setPlanningMode(3);
            manip_planner.setPlanningMode(3);
            send_status(3,0,0,'KeyframePlanners:TELEOP MODE');
        elseif(modeset.mode==drc.manip_plan_control_t.FIXEDJOINTS)
          reaching_planner.setPlanningMode(4);
          manip_planner.setPlanningMode(4);
          send_status(3,0,0,'KeyframePlanners:FIXEDJOINTS MODE');
        end
    end
    
    x=manip_plan_initseed_toggle_listener.getNextMessage(msg_timeout);
    if(~isempty(x))
        disp('Manip Planner init seed toggle msg received .');
        manip_planner.toggleInitSeed(x.mode);
    end
    
    % Pose Goals
    % ----------------------------------------
    rep = getNextMessage(rh_ee,msg_timeout);
    if (~isempty(rep))
        disp('Right hand goal received.');
        p=rep(2:4);   rpy=rep(5:7);
        %    q=rep(5:8);rpy = quat2rpy(q);
        rh_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.rh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    lep = getNextMessage(lh_ee,msg_timeout);
    if (~isempty(lep))
        disp('Left hand goal received.');
        p=lep(2:4);   rpy=lep(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        lh_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.lh = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    rfep = getNextMessage(rf_ee,msg_timeout);
    if (~isempty(rfep))
        disp('Right foot goal received.');
        p=rfep(2:4);   rpy=rfep(5:7);
        %    q=rep(5:8);rpy = quat2rpy(q);
        rf_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.rf = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    lfep = getNextMessage(lf_ee,msg_timeout);
    if (~isempty(lfep))
        disp('Left foot goal received.');
        p=lfep(2:4);   rpy=lfep(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        lf_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.lf = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    hep = getNextMessage(h_ee,msg_timeout);
    if (~isempty(hep))
        disp('head goal received.');
        p = hep(2:4);
        rpy = hep(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        h_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.h = 0; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    % Orientation Goals
    % ----------------------------------------
    hep_orient = getNextMessage(h_ee_orientation,msg_timeout);
    if (~isempty(hep_orient))
        disp('head goal received.');
        p = nan(3,1);
        rpy = hep_orient(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        h_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.h = 1; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    lep_orient = getNextMessage(lh_ee_orientation,msg_timeout);
    if (~isempty(lep_orient))
        disp('left hand orientation goal received.');
        p = nan(3,1);
        rpy = lep_orient(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        lh_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.lh = 1; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    rep_orient = getNextMessage(rh_ee_orientation,msg_timeout);
    if (~isempty(rep_orient))
        disp('right hand orientation goal received.');
        p = nan(3,1);
        rpy = rep_orient(5:7);
        %    q=lep(5:8);rpy = quat2rpy(q);
        rh_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.rh = 1; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    % Gaze Goals
    % ----------------------------------------
    hep_gaze = getNextMessage(h_ee_gaze,msg_timeout);
    if (~isempty(hep_gaze))
        disp('head gaze goal received.');
        p = hep_gaze(2:4);
        rpy = nan(3,1);
        h_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.h = 2; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    hep_lidar_gaze = getNextMessage(lidar_ee_gaze,msg_timeout);
    if (~isempty(hep_lidar_gaze))
        disp('head lidar gaze goal received.');
        p = hep_lidar_gaze(2:4);
        rpy = nan(3,1);
        lidar_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.lidar = 2; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    lep_gaze = getNextMessage(lh_ee_gaze,msg_timeout);
    if (~isempty(lep_gaze))
        disp('left hand gaze goal received.');
        p = lep_gaze(2:4);
        rpy = nan(3,1);
        lh_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.lh = 2; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    rep_gaze = getNextMessage(rh_ee_gaze,msg_timeout);
    if (~isempty(rep_gaze))
        disp('right hand gaze goal received.');
        p = rep_gaze(2:4);
        rpy = nan(3,1);
        rh_ee_goal = [p(:); rpy(:)];
        ee_goal_type_flags.rh = 2; % 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    [x,ts] = getNextMessage(atlas_state_frame,msg_timeout);
    if (~isempty(x))
        %  fprintf('received state at time %f\n',ts);
        % disp('Robot state received.');
        % note: setting the desired to actual at
        % the start of the plan might cause an impulse from gravity sag
        x0 = zeros(robot.getNumStates,1);
%         x0(aff_manager.atlas2robotFrameMap) = x;
%         for i = 1:aff_manager.num_affs
%           if(aff_manager.isCollision(i))
%             x0(aff_manager.aff2robotFrameMap{i}) = aff_manager.aff_state{i};
%           end
%         end
        x0 = x;
    end
    
%    data = getNextMessage(l_hand_listener,msg_timeout);
%    if(~isempty(data))
% %       x0(aff_manager.lhand2robotFrameMap) = data;
%    end
%    
%    data = getNextMessage(r_hand_listener,msg_timeout);
%    if(~isempty(data))
% %       x0(aff_manager.rhand2robotFrameMap) = data;
%    end
    x= constraint_listener.getNextMessage(msg_timeout); % not a frame
    if(~isempty(x))
        num_links = length(x.time);
        lh_name=keyframe_adjustment_engine.get_lhname();
        rh_name=keyframe_adjustment_engine.get_rhname();
        if((num_links==1)&&(strcmp(x.name,lh_name))) 
            disp('received keyframe constraint for left hand');
            lh_ee_constraint = x;
        elseif((num_links==1)&&(strcmp(x.name,rh_name)))
            disp('received keyframe constraint for right hand');
            rh_ee_constraint = x;
        elseif((num_links==1)&&(strcmp(x.name,'l_foot')))
            disp('received keyframe constraint for left foot');
            lf_ee_constraint = x;
            %        x_pos = x.desired_pose;
            %        rpy = quat2rpy(x_pos(4:7));
            %        trans_mat = HT(x_pos(1:3),rpy(1),rpy(2),rpy(3));
            %        lf_ee_pos = trans_mat*[lfoot_pts;ones(1,size(lfoot_pts,2))];
            %        lf_ee_constraint.desired_pose = [lf_ee_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),x_pos(4:7))];
        elseif((num_links==1)&&(strcmp(x.name,'r_foot')))
            disp('received keyframe constraint for right foot');
            rf_ee_constraint = x;
            %        x_pos = x.desired_pose;
            %        rpy = quat2rpy(x_pos(4:7));
            %        trans_mat = HT(x_pos(1:3),rpy(1),rpy(2),rpy(3));
            %        rf_ee_pos = trans_mat*[rfoot_pts;ones(1,size(rfoot_pts,2))];
            %        rf_ee_constraint.desired_pose = [rf_ee_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),x_pos(4:7))];
        elseif((num_links==1)&&(strcmp(x.name,'head')))
            disp('received keyframe constraint for head');
            h_ee_constraint = x;
        elseif((num_links==1)&&(strcmp(x.name,'pelvis')))
            disp('received keyframe constraint for pelvis');
            pelvis_constraint = x;
        elseif((num_links==1)&&(strcmp(x.name,'com::')))
            disp('received keyframe constraint for CoM');
            com_constraint = x;
        else
            disp('Keyframe adjustment engine currently expects one constraint at a time') ;
        end
        displayGazeConstraint(ee_goal_type_flags);
        keyframe_adjustment_engine.adjustAndPublishCachedPlan(x0,rh_ee_constraint,lh_ee_constraint,lf_ee_constraint,rf_ee_constraint,h_ee_constraint,pelvis_constraint,com_constraint,ee_goal_type_flags);
        
        % clear old constraints (currently we maintain a buffer of h constraint, it persists)
        lh_ee_constraint = [];
        rh_ee_constraint = [];
        lf_ee_constraint = [];
        rf_ee_constraint = [];
        pelvis_constraint = [];
        com_constraint = [];
    end
    
    x= plan_adjust_and_reach_listener.getNextMessage(msg_timeout); % not a frame
    if(~isempty(x))
        q_start = keyframe_adjustment_engine.adjustCachedPlanToCurrentStateAndGetFirstPosture(x0,x.mode);
        %q_start = keyframe_adjustment_engine.getFirstPostureOfCachedPlan();
        useIK_state = 0;
        posture_planner.generateAndPublishPosturePlan(x0,q_start,useIK_state);
        cache = posture_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
       
    x= plan_pelvis_adjust_listener.getNextMessage(msg_timeout); % not a frame
    if(~isempty(x))
        keyframe_adjustment_engine.adjustCachedPlanToCurrentPelvisPose(x0,x.mode);
        %keyframe_adjustment_engine.adjustCachedPlanToCurrentRobotState(x0,x.mode);
    end
    
    x= sse_compensation_listener.getNextMessage(msg_timeout); % not a frame
    if(~isempty(x))
        reaching_planner.generateReachPlanToCompensateForCurrentSSE(x0,x.mode);
        cache = reaching_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    
    [lh_ee_traj,~]= lh_ee_motion_command_listener.getNextMessage(msg_timeout);
    if(~isempty(lh_ee_traj))
        disp('Left hand traj goal received.');
        p = lh_ee_traj(end).desired_pose(1:3);% for now just take the end state
        q = lh_ee_traj(end).desired_pose(4:7);q=q/norm(q);
        lep = [p(:);q(:)];
        rpy = quat2rpy(q);
        lh_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.lh=0;% 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    [rh_ee_traj,~]= rh_ee_motion_command_listener.getNextMessage(msg_timeout);
    if(~isempty(rh_ee_traj))
        disp('Right hand traj goal received.');
        p = rh_ee_traj(end).desired_pose(1:3);% for now just take the end state
        q = rh_ee_traj(end).desired_pose(4:7);q=q/norm(q);
        rep = [p(:);q(:)];
        rpy = quat2rpy(q);
        rh_ee_goal=[p(:);rpy(:)];
        ee_goal_type_flags.rh=0;% 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    [lf_ee_traj,~]= lf_ee_motion_command_listener.getNextMessage(msg_timeout);
    if(~isempty(lf_ee_traj))
        disp('Left foot traj goal received.');
        p = lf_ee_traj(end).desired_pose(1:3);% for now just take the end state
        q = lf_ee_traj(end).desired_pose(4:7);q=q/norm(q);
        rpy = quat2rpy(q);
        trans_mat = HT(p,rpy(1),rpy(2),rpy(3));
        lep_pos = trans_mat*[lfoot_pts,ones(1,size(lfoot_pts,2))];
        lep = [lep_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),q(:))];
        lf_ee_goal=[lep_pos(1:3,:);bsxfun(@times,ones(1,size(lfoot_pts,2)),rpy(1:3))];
        ee_goal_type_flags.lf=0;% 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    [rf_ee_traj,~]= rf_ee_motion_command_listener.getNextMessage(msg_timeout);
    if(~isempty(rf_ee_traj))
        disp('Right hand traj goal received.');
        p = rf_ee_traj(end).desired_pose(1:3);% for now just take the end state
        q = rf_ee_traj(end).desired_pose(4:7);q=q/norm(q);
        rpy = quat2rpy(q);
        trans_mat = HT(p,rpy(1),rpy(2),rpy(3));
        rep_pos = trans_mat*[rfoot_pts,ones(1,size(rfoot_pts,2))];
        rep = [rep_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),q(:))];
        rf_ee_goal=[rep_pos(1:3,:);bsxfun(@times,ones(1,size(rfoot_pts,2)),rpy(1:3))];
        ee_goal_type_flags.rf=0;% 0-POSE_GOAL, 1-ORIENTATION_GOAL, 2-GAZE_GOAL
    end
    
    if( (~isempty(rep))|| (~isempty(lep)) ||(~isempty(rfep))||(~isempty(lfep))||...
            (~isempty(rep_orient))|| (~isempty(lep_orient))||...
            (~isempty(rep_gaze))|| (~isempty(lep_gaze))||...
            (~isempty(rh_ee_traj))||(~isempty(lh_ee_traj))||...
            (~isempty(rf_ee_traj))||(~isempty(lf_ee_traj)))
        displayGazeConstraint(ee_goal_type_flags);
        reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
        cache = reaching_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    [trajoptconstraint,postureconstraint]= trajoptconstraint_listener.getNextMessage(msg_timeout);
    if(~isempty(trajoptconstraint))
        disp('time indexed traj opt constraint for manip plan received .');
        
        % cache the a subset of fields as indices for aff_indexed_plans
        timestamps =[trajoptconstraint.time];
        ee_names = {trajoptconstraint.name};
        ee_loci = zeros(6,length(ee_names));

        % joint_timestamps=[postureconstraint.time];
        % joint_names = {postureconstraint.name};
        % joint_positions = [postureconstraint.joint_position];
        for i=1:length(ee_names),
            p = trajoptconstraint(i).desired_pose(1:3);% for now just take the end state
            q = trajoptconstraint(i).desired_pose(4:7);q=q/norm(q);
            rpy = quat2rpy(q);
            ee_loci(:,i)=[p(:);rpy(:)];
        end
        displayGazeConstraint(ee_goal_type_flags);
        manip_planner.generateAndPublishManipulationPlan(x0,ee_names,ee_loci,timestamps,postureconstraint,ee_goal_type_flags);
        cache = manip_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
 
    indexed_trajoptconstraint= indexed_trajoptconstraint_listener.getNextMessage(msg_timeout);
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
        displayGazeConstraint(ee_goal_type_flags);
        manip_planner.generateAndPublishManipulationMap(x0,ee_names,ee_loci,affIndices,ee_goal_type_flags);
        % NO KEYFRAME ADJUSTMENT FOR MANIP MAP
    end
    
    
    [trajoptconstraint,postureconstraint]= wholebodytrajoptconstraint_listener.getNextMessage(msg_timeout);
    if(~isempty(trajoptconstraint))
        disp('time indexed traj opt constraint for whole body plan received .');
        
        % cache the a subset of fields as indices for aff_indexed_plans
        timestamps =[trajoptconstraint.time];
        ee_names = {trajoptconstraint.name};
        ee_loci = zeros(6,length(ee_names));
        
        % joint_timestamps=[postureconstraint.time];
        % joint_names = {postureconstraint.name};
        % joint_positions = [postureconstraint.joint_position];
        for i=1:length(ee_names),
            p = trajoptconstraint(i).desired_pose(1:3);% for now just take the end state
            q = trajoptconstraint(i).desired_pose(4:7);q=q/norm(q);
            rpy = quat2rpy(q);
            ee_loci(:,i)=[p(:);rpy(:)];
        end
        displayGazeConstraint(ee_goal_type_flags);
        wholebody_planner.generateAndPublishWholeBodyPlan(x0,ee_names,ee_loci,timestamps,postureconstraint,ee_goal_type_flags);
        cache = wholebody_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    posture_goal =preset_posture_goal_listener.getNextMessage(msg_timeout);
    useIK_state = 0;
    if(~isempty(posture_goal))
        disp('Preset Posture goal received .');
        if(posture_goal.preset==drc.robot_posture_preset_t.STANDING_HNDS_DWN)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));%standing hands down
            useIK_state = 1;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.STANDING_HNDS_UP)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_hands_up.mat'));%standing hands up
            useIK_state = 1;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.SITTING_HNDS_DWN)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));%seated hands down
            useIK_state = 0;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.SITTING_HNDS_UP)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'));%seated hands up
            useIK_state = 0;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.PROJECTILE)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_hands_projectile.mat'));%atlas
            useIK_state = 0;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.CROUCHING_HNDS_DWN)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_crouching_fp.mat'));
            useIK_state = 5;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.STANDING_RGTHND_REACH)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_comfortable_right_arm_manip.mat'));
            useIK_state = 0;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.STANDING_BDI_FP)
            d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));%bdi fp
            useIK_state = 0;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.LFTHND_DWN)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
            useIK_state = 3; % A hack, indicate using the left arm joint angles of the mat file
        elseif(posture_goal.preset==drc.robot_posture_preset_t.RGTHND_DWN)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
            useIK_state = 4; % A hack, indicate using the right arm joint angles of the mat file
        elseif(posture_goal.preset==drc.robot_posture_preset_t.LFTHND_INHEADVIEW)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_larm_inhead_view.mat'));
            useIK_state = 3; % A hack, indicate using the left arm joint angles of the mat file
        elseif(posture_goal.preset==drc.robot_posture_preset_t.RGTHND_INHEADVIEW)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_rarm_inhead_view.mat'));
            useIK_state = 4; % A hack, indicate using the right arm joint angles of the mat file
        elseif(posture_goal.preset==drc.robot_posture_preset_t.LEFT_HAND_EXTENDED)
            d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_arms_extended.mat'));
            useIK_state = 1;
        elseif(posture_goal.preset==drc.robot_posture_preset_t.HOSE_MATING)
          d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_hose_mating.mat'));
          useIK_state = 6;
        end
        q_desired = d.xstar(1:getNumDOF(robot));
        if(useIK_state ==1||useIK_state == 3|| useIK_state == 4) % correct pelvis
            q_desired([1 2 6]) = x0([1 2 6]); % For stand hands up/down, change the pelvis orientation to the nominal one
        elseif(useIK_state == 5 || useIK_state == 6)
        else
            q_desired(1:6) = x0(1:6); % fix pelvis pose to current
        end
        posture_planner.generateAndPublishPosturePlan(x0,q_desired,useIK_state);
        cache = posture_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    posture_goal = [];
    while 1
      newer_posture_goal = posture_goal_listener.getNextMessage(msg_timeout);
      if ~isempty(newer_posture_goal)
        posture_goal = newer_posture_goal;
      else
        break
      end
    end
    if(~isempty(posture_goal))
        disp('Posture goal received .');
        q0 = x0(1:getNumDOF(robot));
        q_desired =q0;
        joint_names = {posture_goal.joint_name};
        joint_positions = [posture_goal.joint_position];
        for i=1:length(joint_names),
            dofnum = strcmp(robot.getStateFrame.coordinates,joint_names{i});
            q_desired(dofnum) = joint_positions(i);
        end
        q_desired([1,2,6]) = x0([1,2,6]); % fix pelvis pose to current % THIS IS WRONG, this prevents the robot from squating.
        useIK_state = 2; % Doing IK for all joints, with foot on the ground.
        if(posture_planner.isBDIManipMode())
          useIK_state = 7;
        end
        posture_planner.generateAndPublishPosturePlan(x0,q_desired,useIK_state);
        cache = posture_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    [posegoal,postureconstraint]= pose_goal_listener.getNextMessage(msg_timeout);
    if(~isempty(posegoal))
        disp('pose goal received .');
        
        % cache the a subset of fields as indices for aff_indexed_plans
        timestamps =[posegoal.time];
        ee_names = {posegoal.name};
        ee_loci = zeros(6,length(ee_names));
        
        % joint_timestamps=[postureconstraint.time];
        % joint_names = {postureconstraint.name};
        % joint_positions = [postureconstraint.joint_position];
        for i=1:length(ee_names),
            p = posegoal(i).desired_pose(1:3);% for now just take the end state
            q = posegoal(i).desired_pose(4:7);q=q/norm(q);
            rpy = quat2rpy(q);
            ee_loci(:,i)=[p(:);rpy(:)];
        end
        displayGazeConstraint(ee_goal_type_flags);
        endpose_planner.generateAndPublishCandidateRobotEndPose(x0,ee_names,ee_loci,timestamps,postureconstraint,rh_ee_goal,lh_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
        cache = endpose_planner.getPlanCache();
        keyframe_adjustment_engine.setPlanCache(cache);
    end
    
    %listen to  committed robot plan or rejected robot plan
    % channels and clear flags on plan termination.
    p = committed_plan_listener.getNextMessage(msg_timeout);
    if (~isempty(p))
        disp('candidate manipulation plan was committed');
        if(ee_goal_type_flags.h ~= 2)
            h_ee_constraint = [];
        end
        if(ee_goal_type_flags.lh ~=2)
            lh_ee_goal = [];
        end
        if(ee_goal_type_flags.rh ~=2)
            rh_ee_goal = [];
        end
        rf_ee_goal = [];
        lf_ee_goal = [];
        lh_ee_constraint = [];
        rh_ee_constraint = [];
        lf_ee_constraint = [];
        rf_ee_constraint = [];
        pelvis_constraint = [];
        com_constraint = [];
        %h_ee_goal = [];
    end
    
    p = rejected_plan_listener.getNextMessage(msg_timeout);
    if (~isempty(p))
        disp('candidate manipulation plan was rejected');
        if(ee_goal_type_flags.h ~= 2)
            h_ee_constraint = [];
        end
        if(ee_goal_type_flags.lh ~=2)
            lh_ee_goal = [];
        end
        if(ee_goal_type_flags.rh ~=2)
            rh_ee_goal = [];
        end
        rf_ee_goal = [];
        lf_ee_goal = [];
        lh_ee_constraint = [];
        rh_ee_constraint = [];
        lf_ee_constraint = [];
        rf_ee_constraint = [];
        pelvis_constraint = [];
        com_constraint = [];
        %h_ee_goal = [];
    end
    
    [X,T,G,L]=stored_plan_listener.getNextMessage(msg_timeout);
    if (~isempty(X))
        disp('A stored plan was loaded, updating plan cache in keyframe adjustment engine.');
        keyframe_adjustment_engine.setCacheViaPlanMsg(X,T,G,L);
    end
    
    X = des_arc_speed_listener.getNextMessage(msg_timeout);
    if (~isempty(X))
        send_status(3,0,0,'KeyframePlanners:  desired arc speed was set');
        reaching_planner.setVDesired(X.speed);
        manip_planner.setVDesired(X.speed);
        posture_planner.setVDesired(X.speed);
        endpose_planner.setVDesired(X.speed);
        wholebody_planner.setVDesired(X.speed);
    end
    
    X = des_joint_speed_listener.getNextMessage(msg_timeout);
    if (~isempty(X))
        send_status(3,0,0,'KeyframePlanners:  desired joint speed was set');
        reaching_planner.setQdotDesired(X.speed);
        manip_planner.setQdotDesired(X.speed);
        posture_planner.setQdotDesired(X.speed);
        endpose_planner.setQdotDesired(X.speed);
        wholebody_planner.setQdotDesired(X.speed);
    end
    
    p = getNextMessage (h_ee_clear, msg_timeout);
    if (~isempty(p))
        disp ('Clearing head goal pose for both camera and lidar');
        h_ee_goal = [];
        lidar_ee_goal = [];
        ee_goal_type_flags.h = -1;
        ee_goal_type_flags.lidar = -1;
    end
    
    p = getNextMessage (lh_ee_clear, msg_timeout);
    if (~isempty(p))
        disp ('Clearing left hand goal pose');
        lh_ee_goal = [];
        ee_goal_type_flags.lh = -1;
    end
    
    p = getNextMessage (rh_ee_clear, msg_timeout);
    if (~isempty(p))
        disp ('Clearing right hand  goal pose');
        rh_ee_goal = [];
        ee_goal_type_flags.rh = -1;
    end
    
end

end



% ==============================GRAVE YARD
%  teleop_transform_data = getNextMessage(teleop_transform_mon,0);
%  if(~isempty(teleop_transform_data))
%    display('receive teleop transformation data');
%    teleop_transform_msg = drc.ee_teleop_transform_t(teleop_transform_data);
%    hand2aff_translation = [teleop_transform_msg.hand2aff_offset.translation.x;...
%      teleop_transform_msg.hand2aff_offset.translation.y;...
%      teleop_transform_msg.hand2aff_offset.translation.z];
%    hand2aff_quaternion = [teleop_transform_msg.hand2aff_offset.rotation.w;...
%      teleop_transform_msg.hand2aff_offset.rotation.x;...
%      teleop_transform_msg.hand2aff_offset.rotation.y;...
%      teleop_transform_msg.hand2aff_offset.rotation.z];
%    T_aff_palm = [quat2rotmat(hand2aff_quaternion) hand2aff_translation;0 0 0 1];
%    T_palm_aff = inv_HT(T_aff_palm);
%    if(teleop_transform_msg.ee_type == drc.ee_teleop_transform_t.LEFT_HAND)
%      T_hand_aff = T_hand_palm_l*T_palm_aff;
%    elseif(teleop_transform_msg.ee_type == drc.ee_teleop_transform_t.RIGHT_HAND)
%      T_hand_aff = T_hand_palm_r*T_palm_aff;
%    end
%    aff2hand_offset = T_hand_aff*[0;0;0;1];
%    aff2hand_offset = aff2hand_offset(1:3);
%    mate_axis = -[teleop_transform_msg.mate_axis.x;teleop_transform_msg.mate_axis.y;teleop_transform_msg.mate_axis.z];
%    manip_planner.generateAndPublishSpiralMatingPlan(x0,1,0,aff2hand_offset,mate_axis,h_ee_goal,ee_goal_type_flags);
%  end
%  ee_teleop_data = getNextMessage(teleop_mon,0);
%  if(~isempty(ee_teleop_data))
%    display('receive ee teleop command');
%    ee_teleop_msg = drc.ee_cartesian_adjust_t(ee_teleop_data);
%    ee_delta_pos = [ee_teleop_msg.pos_delta.x;ee_teleop_msg.pos_delta.y;ee_teleop_msg.pos_delta.z];
%    ee_delta_rpy = [ee_teleop_msg.rpy_delta.x;ee_teleop_msg.rpy_delta.y;ee_teleop_msg.rpy_delta.z];
%    q0 = x0(1:getNumDOF(r));
%    if(isempty(aff2hand_offset)||isempty(mate_axis))
%      warning('The teleop transformation is not set yet');
%    else
%      manip_planner.generateAndPublishTeleopPlan(x0,ee_delta_pos,ee_delta_rpy,ee_teleop_msg.RIGHT_HAND,ee_teleop_msg.LEFT_HAND,aff2hand_offset,mate_axis);
%    end
%  end

function displayGazeConstraint(ee_goal_type_flags)
if(ee_goal_type_flags.h == 2)
  msg = 'head camera gaze goal';
  send_status(3,0,0,msg);
end
if(ee_goal_type_flags.lidar == 2)
  msg = 'lidar gaze goal';
  send_status(3,0,0,msg);
end
if(ee_goal_type_flags.lh == 2)
  msg = 'left hand gaze goal';
  send_status(3,0,0,msg);
end
if(ee_goal_type_flags.rh == 2)
  msg = 'right hand gaze goal';
  send_status(3,0,0,msg);
end
end
