function runIngressStateMachine(options)

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

robot_options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),robot_options);
r = setTerrain(r,DRCTerrainMap(@MapWrapperRobot));
r = compile(r);

if(nargin<1) options = struct(); end
if(~isfield(options,'use_mex')) options.use_mex = true; end
if(~isfield(options,'debug')) options.debug = false; end

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingController('standing',r);
quasistatic_controller = QuasistaticMotionController('qs_motion',r,options);

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


