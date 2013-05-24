function runWalkingManipStateMachine(options)

addpath(fullfile(pwd,'frames'));
addpath(fullfile(getDrakePath,'examples','ZMP'));

robot_options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),robot_options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = setTerrain(r,DRCTerrainMap(true,struct('name','WalkingManipStateMachine')));
r = compile(r);

if(nargin<1) options = struct(); end
if(~isfield(options,'use_mex')) options.use_mex = true; end
if(~isfield(options,'debug')) options.debug = false; end

harness_controller = HarnessController('harnessed',r);
standing_controller = StandingManipController('standing',r,struct('control','SimplePD'));
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