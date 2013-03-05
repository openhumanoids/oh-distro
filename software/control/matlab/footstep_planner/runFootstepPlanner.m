function runFootstepPlanner
%NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', options);
%r = Atlas('../drake/examples/Atlas/urdf/atlas_minimal_contact.urdf', options);
biped = Biped(r);

p = FootstepPlanner(biped);
p.run()


end