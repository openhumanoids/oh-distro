%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

%% get affordance fits


if 0
% [wall,~] = lcm_mon.getWallAndDrillAffordances();
% wall.targets = wall.targets(:,1:2);
wall.normal = [1;0;0];
x = .8;
z = 0;
wall_z = .6;
wall_y = .6;
wall_center = [x; .1; .3   + z]; % -7,3,2 works for backwards hand
wall.targets = [wall_center + [0;wall_y/2;wall_z/2]]; 
wall.targets = [wall.targets wall_center + [0;-wall_y/2;wall_z/2]];
wall.targets = [wall.targets wall_center + [0;-wall_y/2;-wall_z/2]];
wall.targets = [wall.targets wall_center + [0;wall_y/2;-wall_z/2]];
drill.drill_axis = [1;0;0];
drill.guard_pos = [.25;-.25;0];  
% drill.drill_axis = [0;0;-1];
% drill.guard_pos = [0;-.25;-.25];
end

use_simulated_state = false;
useVisualization = true;
publishPlans = true;
useRightHand = false;
allowPelvisHeight = true;
lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

[wall,drill] = lcm_mon.getWallAndDrillAffordances();
while isempty(wall) || isempty(drill)
  [wall,drill] = lcm_mon.getWallAndDrillAffordances();
end

drill_pub = drillPlanner(r,atlas,drill.guard_pos, drill.drill_axis,...
  wall.normal, useRightHand, useVisualization, publishPlans, allowPelvisHeight);
triangle = wall.targets;
drill_points = [triangle triangle(:,1)];

q_check = zeros(34,1);
q_check(drill_pub.joint_indices) = .1*randn(9,1);

%% update drill
drill = lcm_mon.getDrillAffordance();
drill_pub = drill_pub.updateDrill(drill.guard_pos, drill.drill_axis);
drill.drill_axis
%% update wall
wall = lcm_mon.getWallAffordance();
drill_pub = drill_pub.updateWallNormal(wall.normal);
wall.targets
triangle = wall.targets;
drill_points = [triangle triangle(:,1)];

%% get nominal posture and publish walking plan
q0_init = [zeros(6,1); 0.0355; 0.0037; 0.0055; zeros(12,1); -1.2589; 0.3940; 2.3311; -1.8152; 1.6828; zeros(6,1); -0.9071;0];

tri_centroid = mean(triangle,2);
q0_init(1:3) = tri_centroid - wall.normal*.5 - [0;0;.5];
q0_init(6) = atan2(wall.normal(2), wall.normal(1));
[xtraj_nominal,snopt_info_nominal,infeasible_constraint_nominal] = drill_pub.findDrillingMotion(q0_init, drill_points, true);


%% move the arm before walking

if use_simulated_state
  q_wall = xtraj_nominal.eval(0);
  q_wall = q_wall(1:34);
else
  q_wall = lcm_mon.getStateEstimate();
end

qf = xtraj_nominal.eval(0);
qf = qf(1:34);
posture_index = setdiff((1:r.num_q)',[drill_pub.joint_indices]');
qf(posture_index) = q_wall(posture_index);
kinsol = r.doKinematics(qf);
drill_f = r.forwardKin(kinsol,drill_pub.hand_body,drill_pub.drill_pt_on_hand);

% [xtraj_arm_init, snopt_info_arm_init, infeasible_constraint_arm_init] = drill_pub.createGotoPlan(q_wall,qf,3);
[xtraj_arm_init,snopt_info_arm_init,infeasible_constraint_arm_init] = drill_pub.createInitialReachPlan(q_wall, drill_f - .1*wall.normal, 5);

%% now we've walked up, lets double check
if use_simulated_state
  q_check = xtraj_arm_init.eval(0);
  q_check = q_check(1:34);f
%   q_check = zeros(34,1);
%   q_check(drill_pub.joint_indices) = .1*randn(9,1);
else
  q_check = lcm_mon.getStateEstimate();
end

[xtraj_nominal,snopt_info_nominal,infeasible_constraint_nominal] = drill_pub.findDrillingMotion(q_check, drill_points, false);

%% pre-drill movement
if use_simulated_state
  q0 = xtraj_arm_init.eval(xtraj_arm_init.tspan(2));
  q0 = q0(1:34);
else
  q0 = lcm_mon.getStateEstimate();
end

x_drill_reach = wall.targets(:,1) - .1*wall.normal;

[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = drill_pub.createInitialReachPlan(q0, x_drill_reach, 5);

%% drill in
if use_simulated_state
  q0 = xtraj_reach.eval(xtraj_reach.tspan(2));
  q0 = q0(1:34);
else
  q0 = lcm_mon.getStateEstimate();
end

[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, wall.targets(:,1), 5);

%% setup the state machine
triangle = wall.targets;
drill_points = [triangle triangle(:,1)];
segment_index = 1;
cut_lengths = sum((drill_points(:,2:end) - drill_points(:,1:end-1)).*(drill_points(:,2:end) - drill_points(:,1:end-1)));
[~,diagonal_index] = max(cut_lengths);

short_cut = .03;
long_cut = .1;

%% increment the state machine
if use_simulated_state
  q0 = xtraj_drill.eval(xtraj_drill.tspan(2));
  q0 = q0(1:34);
else
  q0 = lcm_mon.getStateEstimate();
end

kinsol = r.doKinematics(q0);
drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill.guard_pos);

in_goal = norm(drill0 - drill_points(:,segment_index+1)) < .05

if in_goal
  segment_index = segment_index + 1;
  if segment_index == diagonal_index,
    cut_length = short_cut;
  else
    cut_length = long_cut;
  end
else
  cut_length = long_cut;
end

segment_dir = (drill_points(:,segment_index+1) -drill_points(:,segment_index));
segment_dir = segment_dir/norm(segment_dir);

line_param = -(drill_points(:,segment_index) - drill0)'*(drill_points(:,segment_index+1) - drill_points(:,segment_index))/norm(drill_points(:,segment_index+1)-drill_points(:,segment_index))^2;

nearest_point = drill_points(:,segment_index) + line_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));

dist_to_cut = norm(drill0 - nearest_point);
if dist_to_cut < .07
  cut_param = min(1,cut_length/cut_lengths(segment_index) + line_param);
  drill_target = drill_points(:,segment_index) + cut_param*(drill_points(:,segment_index+1) -drill_points(:,segment_index));
else
  drill_target = nearest_point;
end

[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_target, 5);

%%
wall_z = [0;0;1];
wall_z = wall_z - wall_z'*wall.normal*wall.normal;
wall_z = wall_z/norm(wall_z);
wall_y = cross(wall_z, wall.normal);

%% drill delta
% delta = [0;-.02;0];
% delta = wall.normal*.02;
delta = -.1*wall.normal + .0*wall_y - 0.0*wall_z;
if use_simulated_state
  q0 = xtraj_drill.eval(xtraj_drill.tspan(2));
  q0 = q0(1:34);
else
  q0 = lcm_mon.getStateEstimate();
end

kinsol = r.doKinematics(q0);
drill0 = r.forwardKin(kinsol, drill_pub.hand_body, drill.guard_pos);
drill_target = drill0 + delta;
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, drill_target, 5);
