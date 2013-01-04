function runAtlasBalance()

options.floating = true;
r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

nx = r.getNumStates();
nq = nx/2;

x0 = Point(r.getStateFrame);
%xstar = r.manip.resolveConstraints(double(x0));
x0.pelvis_z = 0.928;
xstar = double(x0);

c = COMController(r,xstar(1:nq));

joint_names = r.getStateFrame.coordinates(1:nq);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('atlas', joint_names));
state_listener=LCMCoordinateFrameWCoder('atlas',nx,r.getStateFrame().prefix,lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = ActuatorCmdPublisher('atlas',cmd_names,'ACTUATOR_CMDS');

disp('Balancing controller ready...');
% just run as fast as possible
while (1)
  [x,ts] = getNextMessage(state_listener,1);
  if (~isempty(x))
    t = ts/10;
    u = 0*c.output(t,[],x);

%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u,t*1000000);
%     keyboard;
  end
end

end

