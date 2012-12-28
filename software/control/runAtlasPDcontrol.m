function runAtlasPDcontrol

options.floating = true;
m = RigidBodyModel('../models/mit_gazebo_models/mit_robot_drake/mit_drc_robot_minimal_contact.sdf',options);
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);

nx = r.getNumStates();

B = r.manip.model.B;
idx = B'*(1:nx/2)';

[Kp,Kd] = getPDGains(r); 

% get feedforward and feedback systems
[pdff,pdfb] = pdcontrol(r,Kp,Kd,idx);

% desired position
theta_des = Point(pdff.getInputFrame);
theta_des = double(theta_des);

joint_names = r.getStateFrame.coordinates(1:nx/2);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('mit_drc_robot', joint_names));
state_listener=LCMCoordinateFrameWCoder('mit_drc_robot',nx,r.getStateFrame().prefix,lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = ActuatorCmdPublisher('mit_drc_robot',cmd_names,'ACTUATOR_CMDS');

disp('PD controller ready...');
% just run as fast as possible
while (1)
  [x,ts] = getNextMessage(state_listener,1);
  if (~isempty(x))
    t = ts/10;
    u_ff = pdff.output(t,[],theta_des);
    u_fb = pdfb.output(t,[],x);
    u = u_ff + u_fb;
%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u);
  end
end

end

