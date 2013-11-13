%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

%%
use_simulated_state = true;
useVisualization = true;
publishPlans = true;
use_irobot = true;
use_alt_drill = true;
state_frame = getStateFrame(atlas);
state_frame.subscribe('EST_ROBOT_STATE');

if use_irobot % irobot?
  if use_alt_drill
    drill_pt_on_hand = [.1;-.15;0];
    drill_axis_on_hand = [1;0;0];
    drill_dir_des = [0;1;0];
    drill_dir_threshold = pi;
  else
    drill_pt_on_hand = [0;0;0];
    drill_axis_on_hand = [0;-1;1];
    drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
    drill_dir_des = [1;0;0];
    drill_dir_des = drill_dir_des/norm(drill_dir_des);
    drill_dir_threshold = pi/4;
  end
  
else
  if use_alt_drill
    drill_pt_on_hand = [.1;-.15;0];
    drill_axis_on_hand = [-1+.2;-.5;+.2];
    drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
    drill_dir_des = [0;.2/.5;1];
    drill_dir_threshold = pi;
  else
    drill_pt_on_hand = [0;-.15;0];
    drill_axis_on_hand = [.4;-1;1]; %visual fit from hand
    drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
    drill_dir_des = [-1;-.3;.1];  %just a guess
    drill_dir_des = drill_dir_des/norm(drill_dir_des);
    drill_dir_threshold = pi/4;
  end
end

if ~use_simulated_state
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q0 = x(1:34);
  [lb,ub]=r.getJointLimits;
  q0 = max(min(q0,ub),lb);
%   q0(6) = 0;
  kinsol = r.doKinematics(q0);
  
%   drillpt = [.4;-.8;.4];
%   drillpt = [.3;-.7; .3];
  drillpt = [.5;-.1;.3];
  x_drill_reach = r.forwardKin(kinsol,2,drillpt);
  drilling_world_axis = r.forwardKin(kinsol,2,drillpt + [1;0;0]) - x_drill_reach;
  drilling_world_axis(3) = 0;
  drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
  
  x_drill_in = x_drill_reach + .05*drilling_world_axis;  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
 

  drill_pub = drillTestPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, drill_dir_des, drill_dir_threshold, useVisualization, publishPlans);
  
  first_cut = r.forwardKin(kinsol,2,[0;0;-.1]) - r.forwardKin(kinsol,2,[0;0;0]);
  first_cut = first_cut/norm(first_cut);
  
  horiz_cut_dir = r.forwardKin(kinsol,2,[0;1;0]) - r.forwardKin(kinsol,2,[0;0;0]);
  vert_cut_dir = r.forwardKin(kinsol,2,[0;0;1]) - r.forwardKin(kinsol,2,[0;0;0]);
else
  drilling_world_axis = [1;0;0];
%   drilling_world_axis = [-1;.0;0];
%   x_drill_reach = [.2;-.7;.7]; %works for drilling .4 down, irobot hand
% %   x_drill_reach = [.4;-.1;.4];            %% works for drilling .5 right, irobot hand
% x_drill_reach = [.5;-.6;.7];            %% works for drilling .4 down sandia hand
%     x_drill_reach = [.6;-.3;.3];            %% works for drilling .3 right sandia hand
%     x_drill_reach = [.4;-.5;.4];            %% works for drilling .3 left sandia hand
%     x_drill_reach = [.3;-.3;.4];            %% 45 deg irobot hand, .3
%     drill right

    x_drill_reach = [.5;0;.3];            %% works well for alt drill,
%     both hands


%     x_drill_reach = [.4;-.4;.5];            %% world position of drill reach
%   x_drill_reach = [.5;-.5;.4];            %% world position of drill reach

  x_drill_in = x_drill_reach + [.1;0;0];  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
  drill_pub = drillTestPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, drill_dir_des, drill_dir_threshold, useVisualization, publishPlans);
  
  q0 = zeros(34,1); % get this from robot
  q0(drill_pub.joint_indices(4:end)) = 0*randn(6,1) + 0*[-1.13;.25;2.05;-2;1.6;.47];
first_cut = [0;0;-.1];
  horiz_cut_dir = [0;1;0];
  vert_cut_dir = [0;0;1];
  end
  % first_cut = [0;0;-.1];

  x_drill_line = x_drill_in + first_cut;

first_cut_dir = first_cut/norm(first_cut);
first_cut_dir = [];

% q0(11) = -pi/2;
% q0(drill_pub.joint_indices) = .1*randn(6,1) + q0(drill_pub.joint_indices);
% Create reaching plan
% x_drill_reach = [.4;-.5;.4];          %% world position of drill reach
% x_drill_reach = [.3;-.6;.3]; 
[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q0, x_drill_reach,first_cut_dir, 8);

%% Create drilling in plan
if use_simulated_state
  q_reach_end = xtraj_reach.eval(xtraj_reach.tspan(end));
  q_reach_end = q_reach_end(1:r.num_q);
  [lb,ub]=r.getJointLimits;
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_reach_end = x(1:34);
    q_reach_end = max(min(q_reach_end,ub),lb);

end
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = createDrillingPlan(drill_pub,q_reach_end, x_drill_in,first_cut_dir, 5);

%% circle drill!
if use_simulated_state  
  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
end
[xtraj_circle,snopt_info_circle,infeasible_constraint_circle] = createCircleCutPlan(drill_pub,q_drill_end, x_drill_center, 2);

%% line drill
if use_simulated_state 
  x_drill_line = x_drill_in + .2*horiz_cut_dir + 0.3*vert_cut_dir;

  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
%       q_drill_end = max(min(q_drill_end,ub),lb);

  kinsol = r.doKinematics(q_drill_end);
  x_drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill_pub.drill_pt_on_hand);
  x_drill_line = x_drill0 + -.1*horiz_cut_dir + -.1*vert_cut_dir + .0*drilling_world_axis;

end
% [xtraj_line,snopt_info_line,infeasible_constraint_line]createDirectedLinePlan(obj, q0, x_drill_init, x_drill_final, T)
[xtraj_line,snopt_info_line,infeasible_constraint_line] = createDrillingPlan(drill_pub,q_drill_end, x_drill_line,first_cut_dir, 10);
% [xtraj_line,snopt_info_line,infeasible_constraint_line] = createDirectedLinePlan(drill_pub,q_drill_end, x_drill_line, 20);


%%
if use_simulated_state  
  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
end
x_line_0 = [.4;-.7;.4];  % works for irobot
x_line_1 = x_line_0 + [0;.8;0]; %works for irobot
x_line_0 = [.35;-.6;.8]; %also works for irobot
x_line_1 = x_line_0 + [0;0;-.7];
% x_line_0 = [-.16; -.59;.11];
% x_line_1 = x_line_0 + [0;0;0];
% x_line_0 = [.4;.1;.6]; %works between z=[.55,.8]
% x_line_1 = x_line_0 + [0;-.9;0];
[xtraj_line,snopt_info_line,infeasible_constraint_line] = createLinePlan(drill_pub,q_drill_end, x_line_0,x_line_1, 2);
snopt_info_line

%%
% horizontal line
if use_simulated_state  
  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
end
quat_vert = [sqrt(2)/2; 0; 
  sqrt(2)/2;0];
quat_horiz = [.5;-.5;.5;-.5];
% x_line_0 = [.22;-.75;.66];
x_line_0 = [.5;-.4;.5];
x_line_vert = x_line_0 + [0;0;.4];
x_line_0_horiz = [.5;-.3;.5];  % works with sandia! .5m horizontal travel, axis constrained
x_line_horiz = x_line_0_horiz + [0;.5;0];
% x_drill_end = xtraj_line.eval(0);
I = [1 2 3 6 drill_pub.joint_indices];
q_drill_end(I) = q_drill_end(I) + randn(length(I),1);
% [xtraj_line,snopt_info_line,infeasible_constraint_line] = createConstrainedLinePlan(...
%   drill_pub,q_drill_end, x_line_0,x_line_vert,quat_vert, .3, 2);
[xtraj_line,snopt_info_line,infeasible_constraint_line] = createConstrainedLinePlan(...
  drill_pub,q_drill_end, x_line_0_horiz,x_line_horiz,quat_horiz, .3, 2);

snopt_info_line


%% find initial posture
q0_init = [zeros(6,1); 0.0355; 0.0037; 0.0055; zeros(12,1); -1.2589; 0.3940; 2.3311; -1.8152; 1.6828; zeros(6,1); -0.9071;0];
triangle = [[0;0;.9] [0;0;1.5] [0;.6;.9]];
% triangle = [[0;0;.9] [0;0;1.5] [.6;0;.9]];
drill_points = [triangle triangle(:,1)];
tri_centroid = mean(triangle,2);
q0_init(1:3) = tri_centroid - drilling_world_axis*.5 - [0;0;.5];
q0_init(6) = atan2(drilling_world_axis(2), drilling_world_axis(1));
[xtraj,snopt_info,infeasible_constraint] = drill_pub.findDrillingMotion(q0_init, drill_points, true);