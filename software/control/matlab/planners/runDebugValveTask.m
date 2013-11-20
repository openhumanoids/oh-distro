%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
v = r.constructVisualizer;
%% get affordance fits


use_simulated_state = false;
useVisualization = false;
publishPlans = false;
useRightHand = false;
allowPelvisHeight = false;
lcm_mon = drillTaskLCMMonitor(atlas, useRightHand);

if ~use_simulated_state
 
  valve = lcm_mon.getValveAffordance();
  while isempty(valve)
    valve = lcm_mon.getValveAffordance();
  end
  x_drill_reach = valve.init_pt - .2*valve.normal;
  x_drill_in = valve.init_pt;
  x_drill_center = valve.center;
else
  valve.normal = [1;0;0];
  x_drill_reach = [.5;.5;.4];
  x_drill_in = x_drill_reach + [.2;0;0];
  x_drill_center = x_drill_in + [0;-.15;0];
end
finger_pt_on_hand = [0; 0.2702; 0.015];
finger_axis_on_hand = [0;1;0];


drill_pub = drillPlanner(r,atlas,finger_pt_on_hand, finger_axis_on_hand,...
  valve.normal, useRightHand, useVisualization, publishPlans, allowPelvisHeight);

%% pre-drill movement

if use_simulated_state
  q0 = zeros(34,1);
  q0(drill_pub.joint_indices) = q0(drill_pub.joint_indices) + .1*randn(length(drill_pub.joint_indices),1);
else
  q0 = lcm_mon.getStateEstimate();
end

[xtraj_reach,snopt_info_reach,infeasible_constraint_reach] = drill_pub.createInitialReachPlan(q0, x_drill_reach, 5);

%% drill in
q0 = xtraj_reach.eval(xtraj_reach.tspan(2));
q0 = q0(1:34);
[xtraj_drill,snopt_info_drill,infeasible_constraint_drill] = drill_pub.createDrillingPlan(q0, x_drill_in, 5);

%% make a circle
q0 = xtraj_drill.eval(xtraj_drill.tspan(2));
q0 = q0(1:34);
[xtraj_circ,snopt_info_circ,infeasible_constraint_circ] = drill_pub.createCircularPlan(q0, x_drill_center, 2.4*pi,.05);

%% publish and display
t_reach = xtraj_reach.pp.breaks;
x_reach = xtraj_reach.eval(t_reach);

t_in = xtraj_drill.pp.breaks;
x_in = xtraj_drill.eval(t_in);

t_circ = xtraj_circ.pp.breaks;
x_circ = xtraj_circ.eval(t_circ);

t_all = [t_reach, t_in(2:end) + t_reach(end), t_circ(2:end) + t_reach(end) + t_in(end)];
x_all = [x_reach x_in(:,2:end) x_circ(:,2:end)];
xtraj_all = PPTrajectory(foh(t_all,x_all));
xtraj_all = xtraj_all.setOutputFrame(r.getStateFrame);
v.playback_speed = 5;
v.playback(xtraj_all);

drill_pub.publishTraj(xtraj_all,snopt_info_circ);