function testKeyframePlanner
options.floating = true;
robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model.urdf'),options);
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
nq = robot.getNumDOF;
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

% test reaching planner
x0 = [q_star;0*q_star];
rh_ee_goal = [0.4695;-0.3904;1.0975;-2.9113;0.2081;0.3391];
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
display('Check reach plan');
snopt_info = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Reach plan fails');
end
display('Check touch plan');
rh_ee_goal = [0.4725;-0.3771;1.0117;-2.9113;0.2081;0.3391];
snopt_info = reaching_planner.generateAndPublishReachingPlan(x0,rh_ee_goal,lh_ee_goal,rf_ee_goal,lf_ee_goal,h_ee_goal,lidar_ee_goal,ee_goal_type_flags);
if(snopt_info>10)
  error('Touch plan fails');
end
end