function D = findShoulderPos(map)
  nSph = size(map, 1);
  nSphPerEdge =  round(nSph^(1/3));
  D = zeros(nSph, 1, 'uint16');
  cubeEdge = 5;
  rangeEnd = floor(cubeEdge/2);
  mapSize = nSphPerEdge * [1 1 1];
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
          if isempty(find(ind == sub2ind(mapSize, l, m, n), 1))
            ind((k+3)+(j+2)*cubeEdge+(i+2)*cubeEdge^2) = sub2ind(mapSize, l, m, n);
          end
        end
      end
    end
    ind = ind(ind~=0);
    D(sph) = sum(nnz(map(ind,:)));
  end
end