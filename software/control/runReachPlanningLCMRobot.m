function runReachPlanningLCMRobot
%NOTEST
mode = 0; % 0 = robot, 1 = base
if mode==1
  lcm_url = 'udpm://239.255.12.68:1268?ttl=1';
else
  lcm_url = 'udpm://239.255.76.67:7667?ttl=1';
end
lcm.lcm.LCM.getSingletonTemp(lcm_url);

options.floating = true;
options.dt = 0.001;
r = Atlas('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% create robot plan listener
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
plan_listener = RobotPlanListener('atlas', joint_names, true, 'COMMITTED_ROBOT_PLAN');

disp('Listening for plans...');
waiting = true;
xtraj = [];
while waiting
  xtraj = plan_listener.getNextMessage(1);
  if (~isempty(xtraj))
    disp('Plan received.');
    waiting = false;
  end
end

T = 5.0; % seconds, hard coded for now
dt = 0.1;
ts = 0:dt:T; % plan timesteps

% lame for now
ind = getActuatedJoints(r);
for i=1:size(xstar,2);
  q = xtraj(i,:);
  q_d = q(ind);
end
des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);

% execute plan
disp('Executing plan...');
t=0;
t_offset = -1;
while t <= T
  [x,tsim] = getNextMessage(state_frame,1);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    qd=des_traj.eval(t);
    qd_frame.publish(t,qd,'JOINT_POSITION_CMDS');
  end
end

end