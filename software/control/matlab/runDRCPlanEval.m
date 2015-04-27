function runDRCPlanEval(run_in_simul_mode,atlas_options,walking_options)

if nargin < 1
  run_in_simul_mode = 0;
end
if nargin < 2
  atlas_options = struct();
end
if nargin < 3
  walking_options = struct();
end

atlas_options = applyDefaults(atlas_options, struct('atlas_version', 5, ...
                                                    'hand_right', 'none', ...
                                                    'hand_left', 'none'));
walking_options = applyDefaults(walking_options, struct('navgoal', [0.5;0;0;0;0;0],...
                                                        'num_steps', 4));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
atlas_options.visual = false; % loads faster
atlas_options.floating = true;
atlas_options.ignore_friction = true;
atlas_options.run_in_simul_mode = run_in_simul_mode;

r = DRCAtlas([],atlas_options);
r = setTerrain(r,DRCTerrainMap(true,struct('name','Controller','listen_for_foot_pose',true)));
r = r.removeCollisionGroupsExcept({'heel','toe','midfoot'});
r = compile(r);

if run_in_simul_mode
  mode = 'sim';
else
  mode = 'hardware';
end
planEval = DRCPlanEval(r, mode);
planEval.run();

end
