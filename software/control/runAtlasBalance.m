function runAtlasBalance()

options.floating = true;
m = RigidBodyModel('../models/mit_gazebo_models/mit_robot_drake/mit_drc_robot_minimal_contact.sdf',options);
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);
r = setSimulinkParam(r,'MinStep','0.001');

nx = r.getNumStates();
nq = nx/2;

x0 = Point(r.getStateFrame);
xstar = r.manip.resolveConstraints(double(x0));

c = COMController(r,xstar(1:nq));

joint_names = r.getStateFrame.coordinates(1:nq);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('mit_drc_robot', joint_names));
state_listener=LCMCoordinateFrameWCoder('mit_drc_robot',nx,r.getStateFrame().prefix,lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = ActuatorCmdPublisher('mit_drc_robot',cmd_names,'ACTUATOR_CMDS');

disp('Balancing controller ready...');
% just run as fast as possible
while (1)
  [x,ts] = getNextMessage(state_listener,1);
  if (~isempty(x))
    t = ts/10;
    u = c.output(t,[],x);

%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u);
%     keyboard;
  end
end

end

