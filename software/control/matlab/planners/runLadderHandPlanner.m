%% Setup our simulink objects and lcm monitor
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

doVisualization = true;
doPublish = true;
rbmoptions.floating = true;
if ~doVisualization
  rbmoptions.visual = false;
end
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);

%%
lcm_mon = drillTaskLCMMonitor(atlas, true);

left_hand_axis = [0;sqrt(3)/2;-.5];
right_hand_axis = [0;-sqrt(3)/2;-.5];
left_hand_pt = [0;.1902;.015];
right_hand_pt = -[0;.1902;.015];
planner = ladderHandPlanner(r,atlas,left_hand_axis, left_hand_pt, right_hand_axis,right_hand_pt, ...
        doVisualization, doPublish);
      
%%
while true
  [ctrl_type, ctrl_data] = lcm_mon.getDrillControlMsg();

  switch ctrl_type
    case drc.drill_control_t.LADDER_STRAIGHTEN_LEFT
      ladder = lcm_mon.getLadderAffordance();
      if ~isempty(ladder)
        [q_end, snopt_info, infeasible_constraint] = planner.straightenLeftHand(q0, ladder.forward);
      else
        send_status(4,0,0,'No ladder found, cannot straighten left hand');
      end
    case drc.drill_control_t.LADDER_STRAIGHTEN_RIGHT
      ladder = lcm_mon.getLadderAffordance();
      if ~isempty(ladder)
        [q_end, snopt_info, infeasible_constraint] = planner.straightenRightHand(q0, ladder.forward);
      else
        send_status(4,0,0,'No ladder found, cannot straighten left hand');
      end
  end
end