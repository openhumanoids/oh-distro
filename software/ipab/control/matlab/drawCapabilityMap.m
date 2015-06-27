function drawCapabilityMap(map, options)
  
  diameter = options.diameter;
  nPointsPerSphere = options.nPointsPerSphere;
  sphCenters = options.sphCenters;
  
  nSph = size(map, 1);
  D = zeros(nSph, 1);
  lcmClient = LCMGLClient('CapabilityMap');
  for sph = 1:nSph
    nDir = nnz(map(sph,:));
    D(sph) = nDir/nPointsPerSphere;
    if D(sph) > 0
      color = hsv2rgb([D(sph) 1 1]);
      lcmClient.glColor3f(color(1), color(2), color(3))
      lcmClient.sphere(sphCenters(:,sph), diameter/2, 20, 20);
      
    end
  end
  lcmClient.switchBuffers();
end