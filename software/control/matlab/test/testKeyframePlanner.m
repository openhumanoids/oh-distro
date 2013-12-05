function testKeyframePlanner
warning('off','Drake:RigidBodyManipulator:NoUniqueLink');
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');  
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
options.floating = true;
robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'),options);
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
nq = robot.getNumDOF;
nq_atlas = atlas.getNumDOF();
coords_atlas = atlas.getStateFrame.coordinates(1:nq_atlas);
nomdata = load (strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
q_star = nomdata.xstar(1:nq);
l_hand_frame = handFrame(1,'left');
r_hand_frame = handFrame(1,'right');

hardware_mode = 2;
reaching_planner = ReachingPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % single or multiple/successively specified ee constraints
manip_planner = ManipulationPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % ee motion constraints and point wise IK for manip plans and maps
posture_planner = PosturePlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); %posture and posture preset plans
endpose_planner = EndPosePlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); %search for pose given ee constraints
wholebody_planner = WholeBodyPlanner(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode);%given a time ordered set ee constraints, performs a whole body plan
keyframe_adjustment_engine = KeyframeAdjustmentEngine(robot,atlas,l_hand_frame,...
  r_hand_frame,hardware_mode); % Common keyframe adjustment for all the above planners

rh_reach_goal1 = [0.8539;-0.2184;1.1980;-2.5490;0.2933;0.2897];
rh_touch_goal1 = [0.8189;-0.1764;1.1265;-2.5490;0.2933;0.2897];
rh_touch_goal2 = [0.5885;-0.2675;1.1006;-2.5480;0.2933;0.2897];
% test reaching planner
reaching_planner.setPlanningMode(1);
x0 = [q_star;0*q_star];
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_reach_goal1;
display('Check reach plan');
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Reach plan fails');
end
display('Check touch plan');
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails');
end

display('Check for head gaze constraint');
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1;
h_ee_goal = [0.5634;0;0.9044;NaN;NaN;NaN];
ee_goal_type_flags.h = 2;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails');
end


% check IKSEQUENCE OFF mode
display('Check IKSEQUENCE OFF mode');
reaching_planner.setPlanningMode(2);
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1;
h_ee_goal = [0.5634;0;0.9044;NaN;NaN;NaN];
ee_goal_type_flags.h = 2;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails in IKSEQUENCE OFF mode');
end

display('Check TELEOP mode');
reaching_planner.setPlanningMode(3);
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info == 1)
  error('Touch plan should fail in TELEOP mode');
end

[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal2;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info >10)
  error('Touch plan fail in TELEOP mode');
end

display('Check FIXED JOINT mode');
reaching_planner.setPlanningMode(4);
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1;
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails in FIXED JOINT mode');
end
q_breaks = xtraj(2+(1:nq_atlas),:);
l_arm_ind = cellfun(@(s) ~isempty(strfind(s,'l_arm')),coords_atlas);
if(any(any(abs(diff(q_breaks(l_arm_ind,:),[],2))>1e-3)))
  error('Touch plan does not fix the left arm joint angles');
end

display('Check constraint relaxation in ReachingPlanner')
reaching_planner.setPlanningMode(4);
[rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal();
rh_ee_goal = rh_touch_goal1+[0.2;0.1;0.2;-0.1;0.2;0.2];
reaching_planner.setPosTol(0.04);
reaching_planner.setQuatTol(18);
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails even relax constraint');
end
reaching_planner.setPosTol(0.01);
reaching_planner.setQuatTol(6);
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info<10)
  error('Touch plan should fail even relax constraint');
end
reaching_planner.setPlanningMode(3)
rh_ee_goal = rh_touch_goal1;
reaching_planner.setPosTol(0.06);
reaching_planner.setQuatTol(12);
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('TELEOP should succeed after relaxing constraints');
end
reaching_planner.setPosTol(0.02);
reaching_planner.setQuatTol(6);
[xtraj,snopt_info] = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info<10)
  error('TELEOP should fail using tighter relaxation');
end
% Check PosturePlanner
data = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_standing_larm_inhead_view.mat'));
useIK_state = 3; 
q_desired = data.xstar(1:getNumDOF(robot));
[xtraj,info] = posture_planner.generateAndPublishPosturePlan(x0,q_desired,useIK_state);

% Check EndPosePlanner

end

function [rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags] = clearReachingEEGoal()
rh_ee_goal = [];
lh_ee_goal = [];
rf_ee_goal = [];
lf_ee_goal = [];
h_ee_goal = [];
lidar_ee_goal = [];
ee_goal_type_flags.lh = 0;
ee_goal_type_flags.rh = 0;
ee_goal_type_flags.h = 0;
ee_goal_type_flags.lf = 0;
ee_goal_type_flags.rf = 0;
ee_goal_type_flags.lidar = 0;
end