function runUnpinnedManipStateMachine()

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
options.namesuffix = '';
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
% r = setTerrain(r,DRCTerrainMap(false,struct('name','UnpinnedManipStateMachine')));
r = compile(r);

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingManipController('standing',r,struct('control','Impedance'));
% standing_controller = StandingController('standing',r);
controllers = struct(harness_controller.name,harness_controller,...
                     standing_controller.name,standing_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);

state_machine.run();

end


