function publishRobotStateContinuously(x, channel)
  r = RigidBodyManipulator('');
  r = r.addRobotFromURDF('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', [],[],struct('floating',true));
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(), ...
    'x',JLCMCoder(drc.control.RobotStateCoder(joint_names)));

  starttime = cputime;

  while (1)
  	pause(0.01);
  	publish(robot_state_coder,0,x,channel);
  end

end
