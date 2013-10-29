%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
use_simulated_state = true;

%%
state_frame = getStateFrame(atlas);
state_frame.subscribe('EST_ROBOT_STATE');

if use_simulated_state
  q0 = zeros(34,1); % get this from robot
%   q0(drill_pub.joint_indices) = .1*randn(6,1);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q0 = x(1:34);
  % DELETE THIS CRAP LATER, zeroing joints to avoid calibration issues in
  % legs
%   q0(19) = 0;
%   q0(30) = 0;
%   q0(31) = 0;
end

if ~use_simulated_state
  kinsol = r.doKinematics(q0);
  
  if 0 % irobot?
    drill_pt_on_hand = [0;0;0];
    drill_axis_on_hand = [0;-1;0];
  else
    drill_pt_on_hand = [0;0;0];
    drill_axis_on_hand = [0;0;1];
  end
  x_drill_reach = r.forwardKin(kinsol,5,[.3;-.5;0]);
  drilling_world_axis = r.forwardKin(kinsol,5,[1.3;-.5;0]) - x_drill_reach;
  drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
  
  x_drill_in = x_drill_reach + .1*drilling_world_axis;  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
else
  
  if 0 % irobot?
    drill_pt_on_hand = [0;0;0];
    drill_axis_on_hand = [0;-1;0];
  else
    drill_pt_on_hand = [0;-.2;0];
    drill_axis_on_hand = [0;0;1];
  end
  
  drilling_world_axis = [1;0;0];
%   x_drill_reach = [.3;-.5;.2];            %% world position of drill reach
x_drill_reach = [.3;-.7;.4];            %% world position of drill reach

  x_drill_in = x_drill_reach + [.1;0;0];  %drilling into the wall
  x_drill_center = x_drill_in + [0;.1;0]; % center of drill
end
drill_pub = drillTestPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,drilling_world_axis, true, false);

%% Create reaching plan
x_drill_reach = [.3;-.5;.1];            %% world position of drill reach
[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q0, x_drill_reach, 2);

%% Create drilling in plan
if use_simulated_state
  q_reach_end = xtraj_reach.eval(xtraj_reach.tspan(end));
  q_reach_end = q_reach_end(1:r.num_q);
  [lb,ub]=r.getJointLimits;
  q_reach_end = max(min(q_reach_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_reach_end = x(1:34);
end
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = createDrillingPlan(drill_pub,q_reach_end, x_drill_in, 1);

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
x_drill_line = x_drill_in + [0;.3;0];
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
[xtraj_line,snopt_info_line,infeasible_constraint_line] = createDrillingPlan(drill_pub,q_drill_end, x_drill_line, 2);

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
x_line_0 = [.4;-.8;.4];  % works for irobot
x_line_1 = x_line_0 + [0;.7;0]; %works for irobot
x_line_0 = [.3;-.5;.1];
x_line_1 = x_line_0 + [0;0;.5];
[xtraj_line,snopt_info_line,infeasible_constraint_line] = createLinePlan(drill_pub,q_drill_end, x_line_0,x_line_1, 2);
