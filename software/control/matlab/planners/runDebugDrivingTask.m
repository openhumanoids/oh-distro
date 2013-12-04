%NOTEST
%%
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
%% get affordance fits


useVisualization = true;
doAutoCommit = true;
publishPlans = true;
useRightHand = false;
allowPelvisHeight = false;
lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

finger_pt_on_hand = [0;.3;0];
finger_axis_on_hand = [0; 1; 0];
steer_center_in_root = [.5;.2;.2];
steer_axis_in_root = [1;0;0];
steer_zero_vec_in_root = [0;0;1];
steer_radius = .1;
planner = drivingPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand, ...
        2, steer_center_in_root, steer_axis_in_root, steer_zero_vec_in_root,steer_radius, ...
        useRightHand, useVisualization, publishPlans, doAutoCommit);

      
%%
% q0 = zeros(34,1);
q0 =[
         0
         0
         0
         0
         0
         0
         0
         0
         0
    0.3539
   -1.1304
    2.4001
    2.2191
    1.0654
         0
         0
         0
         0
   -0.0000
         0
   -0.5869
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0
         0];

% [xtraj,snopt_info,infeasible_constraint] = planner.createDrivingPlan(q0, steering_angle, ankle_angle, steering_speed, ankle_speed);
% [xtraj,snopt_info,infeasible_constraint] = planner.createDrivingPlan(q0, steering_angle, ankle_angle, steering_speed, ankle_speed);
[q_vec,steering_vec] = planner.createNominalSteeringPlan(q0, -2*pi, 2*pi);
t_vec = linspace(0,1,length(steering_vec));
xtraj = PPTrajectory(foh(t_vec,[q_vec;0*q_vec]));
xtraj = xtraj.setOutputFrame(r.getStateFrame);

%%
q0 = q_vec(:,100);
% q0 = q0 + .3*randn(34,1);
steering_angle = 2/3*pi;
ankle_angle = 0;
steering_speed = .5;
ankle_speed = .5;
xtraj = planner.createDrivingPlan(q0, steering_angle, ankle_angle, steering_speed, ankle_speed, steering_vec, q_vec);
