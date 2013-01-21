function runAtlasPositionControl

options.floating = true;
r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

nx = r.getNumStates();
nu = r.getNumInputs();

joint_names = r.getStateFrame.coordinates(1:nx/2);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('atlas', joint_names));
state_listener = LCMCoordinateFrameWCoder('atlas',nx,'x',lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = PositionCmdPublisher('atlas',cmd_names,'JOINT_POSITION_CMDS');

disp('Position controller ready...');
while (1)
  [x,t] = getNextMessage(state_listener,1);
  if (~isempty(x))
    u = zeros(nu,1);
    %u(strcmp('r_arm_elx', cmd_names)) = -1.57 + 0.4 * cos(t);
    %u(strcmp('r_arm_ely', cmd_names)) = 0.4 + 0.4 * sin(t);
%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u,t*1000000);
  end
end

end


