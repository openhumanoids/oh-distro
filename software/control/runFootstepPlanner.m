function runFootstepPlanner
%NOTEST

step_length = 0.3;
step_time = 3;

options.floating = true;
options.dt = 0.001;
r = Atlas('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

% set initial conditions in gazebo
% state_frame = r.getStateFrame();
% state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');

p = SplineFootstepPlanner(r, step_length, step_time);
p.run()


end