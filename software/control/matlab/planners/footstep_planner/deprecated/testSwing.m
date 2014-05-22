function testSwing()
% NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
% r = setTerrain(r,DRCTerrainMap());
biped = compile(r);
biped = biped.setTerrain(ExampleTerrain());

last_pos = [0;1.2;0;0;0;0];
next_pos = [.5;2.1;0;0;0;0];

step_traj = planSwing(biped, last_pos, next_pos, struct('step_speed', 0.5, 'step_height', 0.05));

end
