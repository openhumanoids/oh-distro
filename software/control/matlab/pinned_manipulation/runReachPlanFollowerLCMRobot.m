function runReachPlanFollowerLCMRobot
%NOTEST
%mode = 0; % 0 = robot, 1 = base
%if mode==1
%  lcm_url = 'udpm://239.255.12.68:1268?ttl=1';
%else
%  lcm_url = 'udpm://239.255.76.67:7667?ttl=1';
%end
%lcm.lcm.LCM.getSingletonTemp(lcm_url); % only works on mfallons machine

options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);

% create robot plan listener
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
plan_listener = RobotPlanListener('COMMITTED_ROBOT_PLAN',true);

% atlas state subscriber
state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');

disp('Listening for plans...');
waiting = true;
%xtraj = [];
while waiting
  [xtraj,ts] = plan_listener.getNextMessage(10);
  if (~isempty(xtraj))
    disp('Plan received.');
    executeReachPlan(r,state_frame,xtraj,ts);
  end
end


end
