function runAtlasPDcontrol

options.floating = true;
r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

nx = r.getNumStates();
B = r.B;
idx = B'*(1:nx/2)';

[Kp,Kd] = getPDGains(r); 

% get feedforward and feedback systems
[pdff,pdfb] = pdcontrol(r,Kp,Kd,idx);

% desired position
theta_des = Point(pdff.getInputFrame);
theta_des = double(theta_des);

joint_names = r.getStateFrame.coordinates(1:nx/2);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('atlas', joint_names));
state_listener=LCMCoordinateFrameWCoder('atlas',nx,r.getStateFrame().prefix,lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = ActuatorCmdPublisher('atlas',cmd_names,'ACTUATOR_CMDS');

disp('PD controller ready...');
% just run as fast as possible
while (1)
  [x,t] = getNextMessage(state_listener,1);
  if (~isempty(x))
    u_ff = pdff.output(t,[],theta_des);
    u_fb = pdfb.output(t,[],x);
    u = u_ff + u_fb;
%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u,t*1000000);
  end
end

end


