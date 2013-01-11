function runAtlasPinnedReaching

r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf');
options.ground = false;
%v = r.constructVisualizer(options);

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);

c = PinnedReachingControl(sys,r);

nx = r.getNumStates();
nu = r.getNumInputs();

joint_names = r.getStateFrame.coordinates(1:nx/2);

lcmcoder = JLCMCoder(RobotStateCoder('atlas', joint_names));
state_listener = LCMCoordinateFrameWCoder('atlas',nx,'x',lcmcoder);
state_listener.subscribe('EST_ROBOT_STATE');

lcmcoder = JLCMCoder(EndEffectorGoalCoder('atlas', 'r_hand'));
rep_listener = LCMCoordinateFrameWCoder('atlas',3,'x',lcmcoder);
rep_listener.subscribe('R_HAND_GOAL');

lcmcoder = JLCMCoder(EndEffectorGoalCoder('atlas', 'l_hand'));
lep_listener = LCMCoordinateFrameWCoder('atlas',3,'x',lcmcoder);
lep_listener.subscribe('L_HAND_GOAL');

cmd_names = r.getInputFrame().coordinates;
cmd_names = regexprep(cmd_names,'_motor','');     

cmd_publisher = PositionCmdPublisher('atlas',cmd_names,'JOINT_POSITION_CMDS');

disp('Reaching controller ready...');
q_dn = zeros(nu,1);
rep_des = [0.2; -0.3; 0.1];
lep_des = [0.2; 0.3; 0.1];

while (1)
  rep = getNextMessage(rep_listener,1);
  if (~isempty(rep))
    rep_des = rep;
  end
  lep = getNextMessage(lep_listener,1);
  if (~isempty(lep))
    lep_des = lep;
  end
  [x,t] = getNextMessage(state_listener,1);
  if (~isempty(x))
    %v.draw(t,x(1:r.num_q));
    q_dn = c.update(t,rep_des,lep_des,q_dn,x);
    u = c.output(t,q_dn,x);
%     fprintf('time is %f\n',t);
%     fprintf('state is %f\n',x);
%     fprintf('control is %f\n\n',u);
    cmd_publisher.publish(u,t*1000000);
  end
end

end


