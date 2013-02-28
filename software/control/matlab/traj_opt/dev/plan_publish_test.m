options.floating = true;
r = RigidBodyManipulator('../../../../ros_workspace/mit_drcsim_scripts/models/mit_wheeled_stick_torso/model.sdf', options);

nx = r.getNumStates();
B = r.B;
idx = B'*(1:nx/2)';

robot_name = 'wheeled_atlas';
num_knot_points = 11;
time_to_goal = 1;
goal_dist = 5;

joint_names = r.getStateFrame.coordinates(1:nx/2);

lcmcoder = JLCMCoder(RobotStateCoder(robot_name, joint_names));
state_listener = LCMCoordinateFrameWCoder(robot_name, nx, r.getStateFrame().prefix, lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

plan_publisher = RobotPlanPublisher(robot_name, joint_names, true, 'CANDIDATE_ROBOT_PLAN', num_knot_points);

ndx = 1;
joint_names
while (1)
  [x, t] = state_listener.getNextMessage(1);
  if (~isempty(x))
    planned_states = repmat(x, 1, num_knot_points);
    planned_times = linspace(0, time_to_goal, num_knot_points);
    planned_states(ndx,:) = linspace(x(ndx), x(ndx) + goal_dist, num_knot_points);
    plan_publisher.publish(planned_times, planned_states);
  end
end
