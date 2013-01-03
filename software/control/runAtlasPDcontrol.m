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
  [x,ts] = getNextMessage(state_listener,1);
  if (~isempty(x))
    t = ts/10;
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


function [Kp,Kd] = getPDGains(r)

B = r.B;
idx = B'*(1:r.getNumStates()/2)';

Kp = Point(CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}}));
Kd = Point(CoordinateFrame('q_d',length(idx),'d',{r.getStateFrame.coordinates{idx}}));

Kp.l_leg_uhz = 5.0;
Kp.l_leg_mhx = 100.0;
Kp.l_leg_lhy = 2000.0;
Kp.l_leg_kny = 1000.0;
Kp.l_leg_uay = 900.0;
Kp.l_leg_lax = 300.0;
Kp.r_leg_uhz = 5.0;
Kp.r_leg_mhx = 100.0;
Kp.r_leg_lhy = 2000.0;
Kp.r_leg_kny = 1000.0;
Kp.r_leg_uay = 900.0;
Kp.r_leg_lax = 300.0;
Kp.l_arm_usy = 500.0;
Kp.l_arm_shx = 200.0;
Kp.l_arm_ely = 50.0;
Kp.l_arm_elx = 50.0;
Kp.l_arm_uwy = 1.0;
Kp.l_arm_mwx = 2.0;
Kp.r_arm_usy = 500.0;
Kp.r_arm_shx = 200.0;
Kp.r_arm_ely = 50.0;
Kp.r_arm_elx = 50.0;
Kp.r_arm_uwy = 1.0;
Kp.r_arm_mwx = 2.0;
Kp.neck_ay = 20.0;
Kp.back_lbz = 20.0;
Kp.back_mby = 2000.0;
Kp.back_ubx = 800.0;

Kd.l_leg_uhz = 0.01;
Kd.l_leg_mhx = 1.0;
Kd.l_leg_lhy = 10.0;
Kd.l_leg_kny = 10.0;
Kd.l_leg_uay = 5.0;
Kd.l_leg_lax = 2.0;
Kd.r_leg_uhz = 0.01;
Kd.r_leg_mhx = 1.0;
Kd.r_leg_lhy = 10.0;
Kd.r_leg_kny = 10.0;
Kd.r_leg_uay = 5.0;
Kd.r_leg_lax = 2.0;
Kd.l_arm_usy = 3.0;
Kd.l_arm_shx = 20.0;
Kd.l_arm_ely = 3.0;
Kd.l_arm_elx = 3.0;
Kd.l_arm_uwy = 0.1;
Kd.l_arm_mwx = 0.2;
Kd.r_arm_usy = 3.0;
Kd.r_arm_shx = 20.0;
Kd.r_arm_ely = 3.0;
Kd.r_arm_elx = 3.0;
Kd.r_arm_uwy = 0.1;
Kd.r_arm_mwx = 0.2;
Kd.neck_ay = 1.0;
Kd.back_lbz = 0.1;
Kd.back_mby = 2.0;
Kd.back_ubx = 1.0;

Kp = diag(double(Kp));
Kd = diag(double(Kd));

end

