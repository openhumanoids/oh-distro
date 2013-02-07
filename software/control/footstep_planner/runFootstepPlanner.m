function runFootstepPlanner
%NOTEST

step_length = 0.3;
step_rot = pi/8;
step_time = 3;

options.floating = true;
options.dt = 0.001;
r = Atlas('../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', options);

p = FootstepPlanner(r, step_length, step_rot, step_time);
p.run()


end