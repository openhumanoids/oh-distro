function runWalkingStateMachine()

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
% options.floating = false;
% options.namesuffix = 'pinned';
% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
% r = removeCollisionGroupsExcept(r,{'heel','toe'});
% r = compile(r);

options.namesuffix = '';
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
%r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap(@MapWrapperRobot,struct('name','WalkingStateMachine')));
r = compile(r);

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingController('standing',r);
walking_controller = WalkingController('walking',r);

controllers = struct(harness_controller.name,harness_controller, ...
                      standing_controller.name,standing_controller, ...
                      walking_controller.name,walking_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);

monitor = drake.util.MessageMonitor(drc.utime_t,'utime');
lc = lcm.lcm.LCM.getSingleton();
lc.subscribe('PRECOMP_SERVER_HEARTBEAT',monitor);
disp('Waiting for precomputation server heartbeat...');
while isempty(monitor.getNextMessage(10)) end
disp('precomp server running.');

state_machine.run();

end


