function runIngressStateMachine()

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = setTerrain(r,DRCTerrainMap());
r = compile(r);

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingController('standing',r);
quasistatic_controller = QuasistaticMotionController('qs_motion',r);

controllers = struct(harness_controller.name,harness_controller, ...
                      standing_controller.name,standing_controller, ...
                      quasistatic_controller.name,quasistatic_controller);

state_machine = DRCStateMachine(controllers,harness_controller.name);

monitor = drake.util.MessageMonitor(drc.utime_t,'utime');
lc = lcm.lcm.LCM.getSingleton();
lc.subscribe('PRECOMP_SERVER_HEARTBEAT',monitor);
disp('Waiting for precomputation server heartbeat...');
while isempty(monitor.getNextMessage(10)) end
disp('precomp server running.');

state_machine.run();


end


