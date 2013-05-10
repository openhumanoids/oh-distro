function testScan()


options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
% r = setTerrain(r,DRCTerrainMap());
biped = compile(r);
biped = biped.setTerrain(SampleTerrain());


p0 = [0;0;0;0;0;0];
poses = [.5;0;0;0;0;0];
traj = BezierTraj([p0, poses]);

[lambdas, feasibility] = scanWalkingTerrain(biped, traj);

end
