%%
display('Constructing atlas objects');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
rbmoptions.floating = true;
rbmoptions.visual = false;

r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);
atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),rbmoptions);
%%
lcm_mon = palmGazeLCMMonitor(atlas);

doVisualization = false;
doPublish = true;
allowPelvisHeight = false;

planner = palmGazePlanner(r,atlas, doVisualization, doPublish, allowPelvisHeight);

speed = .05;

display('Ready to receive requests');
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
