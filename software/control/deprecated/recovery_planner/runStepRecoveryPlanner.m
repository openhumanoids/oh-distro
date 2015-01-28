function runStepRecoveryPlanner()
%NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap(true,struct('name','Recovery Plan','status_code',6,'fill', true,'normal_radius',2)));
r = compile(r);
p = StepRecoveryPlanner(r);
p.run(0.1)

end