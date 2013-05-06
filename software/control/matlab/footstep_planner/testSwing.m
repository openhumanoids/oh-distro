function testSwing()


options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
% r = setTerrain(r,DRCTerrainMap());
biped = compile(r);
biped = biped.setTerrain(SampleTerrain())

last_pos = [0;0;0;0;0;0];
next_pos = [.5;0;0;0;0;0];


step_traj = planSwing(biped, last_pos, next_pos)

end
