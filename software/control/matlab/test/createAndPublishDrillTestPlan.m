%NOTEST
%%
use_simulated_state = true;
drill_pt_on_hand = [0;0;0];
drill_axis_on_hand = [0;-1;0];
drilling_world_axis = [1;0;0];
x_drill_reach = [.3;-.5;.2];            %% world position of drill reach
x_drill_in = x_drill_reach + [.1;0;0];  %drilling into the wall
x_drill_center = x_drill_in + [0;.1;0]; % center of drill
drill_pub = drillTestPlanPublisher(drill_pt_on_hand, drill_axis_on_hand,drilling_world_axis);

state_frame = getStateFrame(drill_pub.atlas);
state_frame.subscribe('EST_ROBOT_STATE');

%% Create reaching plan
if use_simulated_state
  q0 = zeros(34,1); % get this from robot
else
  [x,~] = getNextMessage(state_frame,10);
  while (~isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q0 = x(1:34);
end
q0(drill_pub.joint_indices) = .1*randn(6,1);
[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q0, x_drill_reach, 10);

%% Create drilling in plan
if use_simulated_state
  q_reach_end = xtraj_reach.eval(xtraj_reach.tspan(end));
  q_reach_end = q_reach_end(1:drill_pub.r.num_q);
  [lb,ub]=drill_pub.r.getJointLimits;
  q_reach_end = max(min(q_reach_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (~isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_reach_end = x(1:34);
end
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = createDrillingPlan(drill_pub,q_reach_end, x_drill_in, 5);

%% circle drill!
if use_simulated_state  
  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:drill_pub.r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (~isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
end
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = createCircleCutPlan(drill_pub,q_drill_end, x_drill_center, 10);