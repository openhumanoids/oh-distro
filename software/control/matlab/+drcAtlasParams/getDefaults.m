function param_sets = getDefaults(r, use_sim_params)
% Returns a struct which maps param set names like 'walking', 'standing', etc.
% to param set objects, which inherit from atlasParams.Base()
typecheck(r, 'DRCAtlas');

if use_sim_params
  param_sets = struct('standing', drcAtlasParams.StandingSim(r),...
                      'walking', drcAtlasParams.WalkingSim(r),...
                      'recovery', drcAtlasParams.RecoverySim(r),...
                      'manip', drcAtlasParams.ManipSim(r));
  if r.atlas_version == 5
    param_sets.bracing = drcAtlasParams.BracingSim(r);
  else
    warning('Drake:BracingNotAvailable', 'Bracing params only defined for Atlas v5. Bracing behavior will not work with this robot.');
  end
else
  param_sets = struct('standing', drcAtlasParams.StandingHardware(r),...
                      'walking', drcAtlasParams.WalkingHardware(r),...
                      'recovery', drcAtlasParams.RecoveryHardware(r),...
                      'manip', drcAtlasParams.ManipHardware(r));
  if r.atlas_version == 5
    param_sets.bracing = drcAtlasParams.BracingHardware(r);
  else
    warning('Drake:BracingNotAvailable', 'Bracing params only defined for Atlas v5. Bracing behavior will not work with this robot.');
  end
end
  