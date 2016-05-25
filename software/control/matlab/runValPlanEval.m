function runValPlanEval(run_in_simul_mode,val_options,walking_options)

if nargin < 1
  run_in_simul_mode = 0;
end
if nargin < 2
  val_options = struct();
end
if nargin < 3
  walking_options = struct();
end

val_options = applyDefaults(val_options, struct('valkyrie_version', 2, ...
                                                    'hand_right', 'none', ...
                                                    'hand_left', 'none'));
walking_options = applyDefaults(walking_options, struct('navgoal', [0.5;0;0;0;0;0],...
                                                        'num_steps', 4));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
val_options.visual = false; % loads faster
val_options.floating = true;
val_options.ignore_friction = true;
val_options.run_in_simul_mode = run_in_simul_mode;

r = OHValkyrie([],val_options);
r = setTerrain(r,DRCTerrainMap(true,struct('name','Controller','listen_for_foot_pose',false)));
r = r.removeCollisionGroupsExcept({'heel','toe','midfoot_front','midfoot_rear'});
r = compile(r);

if run_in_simul_mode
  mode = 'sim';
else
  mode = 'hardware';
end
planEval = ValPlanEval(r, mode);
planEval.run();

end
