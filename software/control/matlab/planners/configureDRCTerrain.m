function biped = configureDRCTerrain(biped, map_mode, q0)
typecheck(biped, 'Biped');

terrain = biped.getTerrain();
if isa(terrain, 'DRCTerrainMap')
  if map_mode == drc.footstep_plan_params_t.HORIZONTAL_PLANE
    terrain = terrain.setFillPlaneFromConfiguration(biped, q0, true);
    terrain = terrain.overrideNormals(true);
    terrain = terrain.overrideHeights(true);
  elseif map_mode == drc.footstep_plan_params_t.FOOT_PLANE
    terrain = terrain.setFillPlaneFromConfiguration(biped, q0, false);
    terrain = terrain.overrideNormals(true);
    terrain = terrain.overrideHeights(true);
  elseif map_mode == drc.footstep_plan_params_t.TERRAIN_HEIGHTS_Z_NORMALS
    terrain = terrain.setFillPlaneFromConfiguration(biped, q0, true);
    terrain = terrain.overrideNormals(true);
    terrain = terrain.overrideHeights(false);
  elseif map_mode == drc.footstep_plan_params_t.TERRAIN_HEIGHTS_AND_NORMALS
    terrain = terrain.setFillPlaneFromConfiguration(biped, q0, false);
    terrain = terrain.overrideNormals(false);
    terrain = terrain.overrideHeights(false);
  end
end
biped = biped.setTerrain(terrain);
biped = compile(biped);
end