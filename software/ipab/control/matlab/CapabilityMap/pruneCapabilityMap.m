function [map, reachabilityIndex, sphCenters]  = pruneCapabilityMap(mapFile, sagittalAngle,...
    transverseAngle, sagittalWeight, transverseWeight, reachabilityWeight)
  
  load(mapFile)
  
  Dmax = max(reachabilityIndex);
  nSph = length(map);
  indices = [];
  
  for sph = 1:nSph
    sa = atan2(sphCenters(3,sph), sphCenters(1,sph));
    ta = atan2(sphCenters(2,sph), sphCenters(1,sph));
    sagittalCost = sagittalWeight * abs(sa - sagittalAngle);
    transverseCost = transverseWeight * abs(ta - transverseAngle);
    reachabilityCost = reachabilityWeight * (Dmax - reachabilityIndex(sph));
    if sqrt(sagittalCost^2 + transverseCost^2) + reachabilityCost < 2
      indices(end + 1) = sph;
    end
  end  
  reachabilityIndex = reachabilityIndex(indices);
  map = map(indices, :);
  sphCenters = sphCenters(:, indices);
end