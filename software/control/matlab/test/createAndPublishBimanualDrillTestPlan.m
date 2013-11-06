%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

%%
% new plan!
% (1) pre-saved grasp pose
% (2) goto pose
% (3) close hands
% (4) goto pre-drill, drill, line, all constrained appropriately
use_simulated_state = false;
useVisualization = true;
publishPlans = true;
use_irobot = false;
state_frame = getStateFrame(atlas);
state_frame.subscribe('EST_ROBOT_STATE');

if use_irobot % irobot?
  drill_pt_on_hand = [0;0;0];
  drill_axis_on_hand = [0;-1;0];
  drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
else
  drill_pt_on_hand = [0;-.15;0];
  drill_axis_on_hand = [.4;-1;1]; %visual fit from hand
  drill_axis_on_hand = [0;-1;0];
  drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
end

if ~use_simulated_state
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q0 = x(1:34);
  [lb,ub]=r.getJointLimits;
  q0 = max(min(q0,ub),lb);

  kinsol = r.doKinematics(q0);
  
  drillpt = [.4; 0; .2];
  x_drill_reach = r.forwardKin(kinsol,2,drillpt);
  drilling_world_axis = r.forwardKin(kinsol,2,drillpt + [1;0;0]) - x_drill_reach;
  drilling_world_axis(3) = 0;
  drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
  
  x_drill_in = x_drill_reach + .1*drilling_world_axis;  %drilling into the wall 
  
  drill_pub = simpleBimanualDrillPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, useVisualization, publishPlans);
  
  horiz_cut_dir = r.forwardKin(kinsol,2,[0;1;0]) - r.forwardKin(kinsol,2,[0;0;0]);
  vert_cut_dir = r.forwardKin(kinsol,2,[0;0;1]) - r.forwardKin(kinsol,2,[0;0;0]);

else
  drilling_world_axis = [1;0;0];
  x_drill_reach = [.4;0;.4];            %% world position of drill reach
  horiz_cut_dir = [0;1;0];
  vert_cut_dir = [0;0;1];
  x_drill_in = x_drill_reach + [.1;0;0];  %drilling into the wall
  drill_pub = simpleBimanualDrillPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand,...
    drilling_world_axis, useVisualization, publishPlans);
  
  q0 = zeros(34,1); % get this from robot
  q0(drill_pub.joint_indices) = .1*randn(15,1);
end

% [xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q0, x_drill_reach,first_cut_dir, 10);

%%
qgrasp=[  -0.0033
   -0.0536
    0.8906
    0.0103
    0.0296
   -1.7595
    0.0116
    0.0914
   -0.0013
   -0.7153
   -0.2432
    1.9372
    1.5427
    1.9290
   -0.0009
    0.0488
   -0.3764
    0.7594
   -0.3872
   -0.0521
   -0.4819
   -1.0210
    1.3206
    1.9855
   -1.3892
    1.3208
    0.0003
   -0.0488
   -0.3751
    0.7596
   -0.3835
    0.0522
    1.1833 - .0053;
    0.0140];
       
 [xtraj_init,snopt_info_init,infeasible_constraint_init] = createGotoPosePlan(drill_pub, q0, qgrasp, 5);
 
 %%
 if use_simulated_state
  q_init_end = xtraj_init.eval(xtraj_init.tspan(end));
  q_init_end = q_init_end(1:r.num_q);
  [lb,ub]=r.getJointLimits;
  q_init_end = max(min(q_init_end,ub),lb);
else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_init_end = x(1:34);
 end

[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = createInitialReachPlan(drill_pub, q_init_end, x_drill_reach, 2);

%%
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

[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = createDrillingPlan(drill_pub, q_reach_end, x_drill_in, 2);
snopt_info_drill
 %%
%% line drill
if use_simulated_state 
  x_drill_line = x_drill_in + horiz_cut_dir*.1;

  q_drill_end = xtraj_drill.eval(xtraj_drill.tspan(end));
  q_drill_end = q_drill_end(1:r.num_q);
  q_drill_end = max(min(q_drill_end,ub),lb);
  x_line = x_in + [0;-.4;0];

else
  [x,~] = getNextMessage(state_frame,10);
  while (isempty(x))
    [x,~] = getNextMessage(state_frame,10);
  end
  q_drill_end = x(1:34);
  kinsol = r.doKinematics(q_drill_end);
  x_drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill_pub.drill_pt_on_hand);
  x_line = x_drill0 + horiz_cut_dir*.1;

end

[xtraj_line,snopt_info_line,infeasible_constraint_line] = createDrillingPlan(drill_pub, q_drill_end, x_line, 2);
snopt_info_line
%        %%
% x_line_0 = [.6;.2;.4];
% x_line_1 = x_line_0 + [0;-.4;0];
% [xtraj,snopt_info,infeasible_constraint] = createLinePlan(drill_pub, q0, x_line_0, x_line_1, 1);
% snopt_info