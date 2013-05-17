function publishRobotState(x, channel)
  r = RigidBodyManipulator('');
  r = r.addRobotFromURDF('../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', [],[],struct('floating',true));
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(), ...
    'x',JLCMCoder(RobotStateCoder('atlas', joint_names)));

  publish(robot_state_coder,0,x,channel);
end
