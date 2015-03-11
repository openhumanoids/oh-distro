function runAtlasControllerStandalone(run_in_simul_mode, atlas_version, ctrl_options)

if nargin < 1
  run_in_simul_mode = 0;
end
if nargin < 2
  atlas_version = 4;
end
if nargin < 3
  ctrl_options = struct();
end
ctrl_options = applyDefaults(ctrl_options, struct('atlas_command_channel', 'ATLAS_COMMAND'));

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

threadedControllermex(control.data_mex_ptr, ctrl_options);