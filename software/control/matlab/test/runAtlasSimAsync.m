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

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar = r.resolveConstraints(xstar);
xstar(3) = xstar(3) + 0.03;
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.01;

x0 = xstar;

% Make our Atlas listen for ATLAS_COMMAND over LCM and publish EST_ROBOT_STATE
% command_to_effort = atlasControllers.AtlasCommandToEffortBlock(r);
% command_to_effort = command_to_effort.setInputFrame(drcFrames.AtlasInput(r));
% r = r.setInputFrame(drcFrames.AtlasInput(r));
% output_frame = drcFrames.AtlasState(r);
% output_frame.setMaxRate(2000);
% r = r.setOutputFrame(output_frame);

% sys = cascade(command_to_effort, r);
% sys = r;
sys = cascade(CommandReceiver(r), r);
sys = cascade(sys, StatePublisher(r));

output_select(1).system=1;
output_select(1).output=1;
v = v.setInputFrame(sys.getOutputFrame());
sys = mimoCascade(sys,v,[],[],output_select);

disp('sim starting');
% simulate(sys, [0, inf], x0, struct('gui_control_interface', true));
runLCM(sys, x0, struct('tspan', [0, inf],...
                        'timekeeper', '',...
                        'gui_control_interface', true));

