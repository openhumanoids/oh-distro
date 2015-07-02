function pos = findShoulderPos(map, options)
  sphCenters = options.sphCenters;
  nSph = size(map, 1);
  nSphPerEdge =  round(nSph^(1/3));
  Dmax = 0;
  cubeEdge = 5;
  rangeEnd = floor(cubeEdge/2);
  mapSize = nSphPerEdge * [1 1 1];
  drawCapabilityMap(map, options)
  for sph = 1:nSph
    fprintf('sphere %d of %d\n', sph, nSph)
    ind = zeros(cubeEdge^3, 1);
    for i = -rangeEnd:rangeEnd
      for j = -rangeEnd:rangeEnd
        for k = -rangeEnd:rangeEnd
          [x, y, z] = ind2sub(mapSize, sph);
          l = min([nSphPerEdge, max([1, x+i])]);
          m = min([nSphPerEdge, max([1, y+j])]);
          n = min([nSphPerEdge, max([1, z+k])]);
          if isempty(find(ind == sub2ind(mapSize, l, m, n)))
            ind((k+3)+(j+2)*cubeEdge+(i+2)*cubeEdge^2) = sub2ind(mapSize, l, m, n);
          end
        end
      end
    end
    ind = ind(ind~=0);
    D = sum(nnz(map(ind,:)));
    if D > Dmax
      Dmax = D;
      sphMax = sph;
    end
    
%     drawTreePoints([sphCenters(:, sph); 1; 0; 0; 0], 'frame', true, 'text', 'cube center');
%     drawTreePoints(sphCenters(:, ind), 'text', 'cube');
  end
  pos = sphCenters(:, sphMax);
  drawTreePoints([pos; 1; 0; 0; 0], 'frame', true, 'text', 'best Sphere');
end