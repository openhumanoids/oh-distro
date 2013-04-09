function runQual1StateMachine()

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingController('standing',r);
walking_controller = WalkingController('walking',r);

controllers = struct(harness_controller.name,harness_controller, ...
                      standing_controller.name,standing_controller, ...
                      walking_controller.name,walking_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);
state_machine.run();

end


