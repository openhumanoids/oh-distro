function runQual2StateMachine()

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
options.floating = false;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

harness_controller = HarnessController('harnessed',r,inf);

controllers = struct(harness_controller.name,harness_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);
state_machine.run();

end


