%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

%%
use_simulated_state = true;
useVisualization = true;
publishPlans = false;
use_irobot = true;
state_frame = getStateFrame(atlas);
state_frame.subscribe('EST_ROBOT_STATE');

if use_irobot % irobot?
  drill_pt_on_hand = [0;0;0];
  drill_axis_on_hand = [0;-1;0];
  drill_dir_des = [1;0;0];
  drill_dir_des = drill_dir_des/norm(drill_dir_des);
  drill_dir_threshold = pi/4;
  lh_on_hand = [-.2; .0; .2];
  lh_grasp_axis = [1;0;0];
else
  drill_pt_on_hand = [0;-.15;0];
  drill_axis_on_hand = [.4;-1;1]; %visual fit from hand
  drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
  drill_dir_des = [-1;-.3;.1];  %just a guess
  drill_dir_des = drill_dir_des/norm(drill_dir_des);
  drill_dir_threshold = pi/8;
end

if ~use_simulated_state
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q0 = x(1:34);

  kinsol = r.doKinematics(q0);
  
  drillpt = [.2;-.7;.5];
  x_drill_reach = r.forwardKin(kinsol,2,drillpt);
  drilling_world_axis = r.forwardKin(kinsol,2,drillpt + [1;0;0]) - x_drill_reach;
  drilling_world_axis(3) = 0;
  drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
  
  x_drill_in = x_drill_reach + .1*drilling_world_axis;  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
 

  drill_pub = drillTestPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, drill_dir_des, drill_dir_threshold, useVisualization, publishPlans);

else
  drilling_world_axis = [1;0;0];
%   x_drill_reach = [.2;-.7;.7]; %works for drilling .4 down, irobot hand
%   x_drill_reach = [.4;-.1;.4];            %% works for drilling .5 right, irobot hand
% x_drill_reach = [.5;-.6;.7];            %% works for drilling .4 down sandia hand
%     x_drill_reach = [.6;-.3;.3];            %% works for drilling .3 right sandia hand
%     x_drill_reach = [.4;-.5;.4];            %% works for drilling .3 left sandia hand

    x_drill_reach = [.2;-.7;.5];            %% world position of drill reach
%   x_drill_reach = [.5;-.5;.4];            %% world position of drill reach

  x_drill_in = x_drill_reach + [.1;0;0];  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
  drill_pub = bimanualDrillPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, lh_on_hand, lh_grasp_axis, useVisualization, publishPlans);
  
  q0 = zeros(34,1); % get this from robot
  q0(drill_pub.joint_indices) = .1*randn(15,1);% + [    0.1795
%     0.4623
%     0.1828
%    -0.3834
%    -0.0079
%    -0.3583];
end

% [xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q0, x_drill_reach,first_cut_dir, 10);

%%
% q0(drill_pub.joint_indices) = .5*randn(15,1);
q0=[
                0
         0
         0
         0
         0
         0
    0.0530
   -0.1938
    0.2502
   -1.5708
   -0.8529
    3.0107
         0
    0.3956
         0
         0
         0
         0
         0
         0
   -0.0801
   -1.5708
    0.9738
    1.7808
   -0.0479
    0.8868
         0
         0
         0
         0
         0
         0
    0.4103
         0];
       
       q0(drill_pub.joint_indices) = q0(drill_pub.joint_indices) + .2*randn(15,1);

x_line_0 = [.6;.2;.4];
x_line_1 = x_line_0 + [0;-.4;0];
[xtraj,snopt_info,infeasible_constraint] = createLinePlan(drill_pub, q0, x_line_0, x_line_1, 1);
snopt_info