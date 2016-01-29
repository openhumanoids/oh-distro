function runDRCInstQP(run_in_simul_mode, atlas_options, ctrl_options)

if nargin < 1
  run_in_simul_mode = 0;
end
if nargin < 2
  atlas_options = struct();
end
if nargin < 3
  ctrl_options = struct();
end
atlas_options = applyDefaults(atlas_options, struct('atlas_version', 5, ...
                                                    'hand_right', 'robotiq_weight_only', ...
                                                    'hand_left', 'robotiq_weight_only'));
ctrl_options = applyDefaults(ctrl_options, struct('atlas_command_channel', 'ATLAS_COMMAND',...
                                                  'robot_behavior_channel', 'ATLAS_BEHAVIOR_COMMAND',...
                                                  'max_infocount', -1));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
atlas_options.visual = false; % loads faster
atlas_options.floating = true;
atlas_options.ignore_friction = true;
atlas_options.run_in_simul_mode = run_in_simul_mode;
atlas_options.use_new_kinsol = true;

r = DRCAtlas([],atlas_options);
r = setTerrain(r,DRCTerrainMap(true,struct('name','Controller','listen_for_foot_pose',false)));
r = r.removeCollisionGroupsExcept({'heel','toe','midfoot_front','midfoot_rear'});
r = compile(r);

control = bipedControllers.InstantaneousQPController(r, drcAtlasParams.getDefaults(r, run_in_simul_mode));

threadedControllermex(control.data_mex_ptr, ctrl_options);
