function runAtlasReachingExample

options.floating = true;
dt = 0.001;
r = TimeSteppingRigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',dt,options);

[Kp,Kd] = getPDGains(r.manip);
sys = pdcontrol(r,Kp,Kd);

c = StandingAndReachingControl(sys,r);
%c = StandingControl(sys,r);

nx = r.getNumStates();
nu = r.getNumInputs();

joint_names = r.getStateFrame.coordinates(1:nx/2);
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

lcmcoder = JLCMCoder(RobotStateCoder('atlas', joint_names));
state_listener = LCMCoordinateFrameWCoder('atlas',nx,'x',lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');


lcmcoder = JLCMCoder(EndEffectorGoalCoder('atlas', 'r_hand'));
ep_listener = LCMCoordinateFrameWCoder('atlas',3,'x',lcmcoder);
ep_listener.subscribe('R_HAND_GOAL');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     
cmd_publisher = PositionCmdPublisher('atlas',cmd_names,'JOINT_POSITION_CMDS');

disp('Reaching controller ready...');
q_dn = zeros(nu,1);
ep_des = [0.415; -0.11; 1.075]; %[0.4; -0.3; 1.1];
while (1)
  ep = getNextMessage(ep_listener,1);
  if (~isempty(ep))
    ep_des = ep;
  end
  [x,ts] = getNextMessage(state_listener,1);
  t = ts/10;
  if (~isempty(x))
    q_dn = c.update(t,ep_des,q_dn,x);
    u = c.output(t,q_dn,x);
%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u,t*1000000);
  end
end

end


