function runWalkingManipStateMachine(options)

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.namesuffix = '';
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
%r = setTerrain(r,DRCTerrainMap(true,struct('name','WalkingManipStateMachine','fill',true)));
r = setTerrain(r,DRCFlatTerrainMap());
r = compile(r);

standing_controller = StandingManipController('standing',r);
walking_controller = WalkingController('walking',r);
bracing_controller = BracingController('bracing',r);

controllers = struct(standing_controller.name,standing_controller, ...
                      walking_controller.name,walking_controller,...     
                      bracing_controller.name,bracing_controller);

state_machine = DRCStateMachine(controllers,standing_controller.name);

state_machine.run();
end
