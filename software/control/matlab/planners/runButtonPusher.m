%NOTEST
%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

lcm_mon = drillTaskLCMMonitor(atlas);

%% get affordance fits
drill = lcm_mon.getDrillAffordance();
while isempty(drill)
  drill = lcm_mon.getDrillAffordance();
end

use_simulated_state = true;
useVisualization = true;
publishPlans = true;
buttonInRightHand = true;

finger_pt_on_hand = [0;.2;0];
finger_axis_on_hand = [0;1;0];


drill_button_pub = drillButtonPlanner(r,atlas,drill.button_pos, drill.button_normal, drill.drill_axis,...
 finger_pt_on_hand, finger_axis_on_hand, buttonInRightHand, useVisualization, publishPlans);

%%
while snopt_info_pre > 10
if use_simulated_state
  [lb ub] = atlas.getJointLimits;
  r_arm_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
  q0 = zeros(34,1);
%   q0(r_arm_joint_indices) = [-2;.5;1;-1;0;-1];q0 = min(max(q0,lb),ub);
%   q0(r_arm_joint_indices) = randn(6,1);q0 = min(max(q0,lb),ub);
  
q0(r_arm_joint_indices) =   [   -0.6410
    1.5479
         1
         0
         0
    1.0842];
%   q0 = min(max(q0,lb),ub);
else
  q0 = lcm_mon.getStateEstimate();
end
v.draw(0,q0);
[xtraj_pre,snopt_info_pre,infeasible_constraint_pre] = drill_button_pub.createPrePokePlan(q0, 5);
end
%%
if use_simulated_state
  q0 = xtraj_pre.eval(xtraj_pre.tspan(2));
  q0 = q0(1:34);
else
  q0 = lcm_mon.getStateEstimate();
end
offset = [0;0;0];
[xtraj,snopt_info,infeasible_constraint] = drill_button_pub.createPokePlan(q0, offset, 5);