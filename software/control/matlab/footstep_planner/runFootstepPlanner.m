function runFootstepPlanner
%NOTEST

addpath(fullfile(pwd,'..'));
addpath(fullfile(pwd,'..','frames'));
addpath(fullfile(pwd,'..','util'));
options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_foot_contact.urdf', options);
%r = Atlas('../drake/examples/Atlas/urdf/atlas_minimal_contact.urdf', options);

p = FootstepPlanner(r);
p.run()


end