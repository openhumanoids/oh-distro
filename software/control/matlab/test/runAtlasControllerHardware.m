function runAtlasStateMachine(controller_type, run_in_simul_mode,atlas_version)

if nargin < 1
  controller_type = 2; % 1: PID, 2: PID+manip params, 3: PD+gravity comp, 4: inverse dynamics
  run_in_simul_mode = 0; % Initialize to support drakeWalking-like controllers
                         % (is this redundant with controller_type?)
                         % Use value 1 for normal, 2 to add weights to the
                         % control model's hands
end
if nargin < 2
  run_in_simul_mode = 0;
end
if nargin < 3
  atlas_version = 4;
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
options.floating = true;
options.ignore_friction = true;
options.run_in_simul_mode = run_in_simul_mode;
options.atlas_version = atlas_version;
if (run_in_simul_mode == 2)
  options.hands = 'robotiq_weight_only';
end

r = DRCAtlas([],options);
r = setTerrain(r,DRCTerrainMap(true,struct('name','Controller','listen_for_foot_pose',false)));
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

control = atlasControllers.InstantaneousQPController(r, drcAtlasParams.getDefaults(r),...
   struct('use_mex', 1));

contact_sensor = zeros(length(r.getManipulator().body), 1);

state_coder = drcFrames.AtlasState(r).lcmcoder;
lc = lcm.lcm.LCM.getSingleton();
input_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input, 'timestamp');
state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
lc.subscribe('QP_CONTROLLER_INPUT', input_monitor);
lc.subscribe('EST_ROBOT_STATE', state_monitor);
disp('controller ready');

threadedControllermex(control.data_mex_ptr);