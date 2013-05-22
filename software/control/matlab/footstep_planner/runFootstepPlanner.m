function runFootstepPlanner
%NOTEST

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','fill', true)));
r = compile(r);

p = FootstepPlanner(r);
p.run()


end
