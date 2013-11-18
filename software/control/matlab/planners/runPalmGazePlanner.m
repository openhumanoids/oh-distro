%%
r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
%%
lcm_mon = palmGazeLCMMonitor(atlas);

doVisualization = false;
doPublish = true;
allowPelvisHeight = false;

planner = palmGazePlanner(r,atlas, doVisualization, doPublish, allowPelvisHeight);

speed = .05;


while(true)
  q0 = lcm_mon.getStateEstimate();

  left_palm_pose = lcm_mon.getLeftPalmPose();
  if ~isempty(left_palm_pose)
    [xtraj,snopt_info,infeasible_constraint] = planner.createGazePlan(q0, left_palm_pose, speed, false);
  end
  
  right_palm_pose = lcm_mon.getRightPalmPose();
  if ~isempty(right_palm_pose)
    [xtraj,snopt_info,infeasible_constraint] = planner.createGazePlan(q0, right_palm_pose, speed, true);
  end

  pause(.01);
end
