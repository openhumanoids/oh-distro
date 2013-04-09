function runFootstepPlanner
%NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

p = FootstepPlanner(r);
p.run()


end