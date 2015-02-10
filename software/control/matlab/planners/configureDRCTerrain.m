function biped = configureDRCTerrain(biped, map_mode, q0)
typecheck(biped, 'Biped');

terrain = biped.getTerrain();
if isa(terrain, 'DRCTerrainMap')
  terrain = terrain.configureFillAndOverride(biped, map_mode, q0);
end
biped = biped.setTerrain(terrain);
biped = compile(biped);
end