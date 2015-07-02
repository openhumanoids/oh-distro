function drawCapabilityMap(map, options, center)
  
  if ~isfield(options, 'drawRobot'), options.drawRobot = true; end
  
  if options.drawRobot
    options.floating = false;
    urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
    r = RigidBodyManipulator(urdf,options);
    v = r.constructVisualizer();
    shoulder = r.findLinkId('RightShoulderAdductor');
    q = zeros(r.num_positions,1);
    q(r.getBody(shoulder).position_num) = -pi/2;
    v.draw(0,q);
    kinsol = r.doKinematics(q);
    center = r.forwardKin(kinsol, shoulder, [0;0;0], 0);
  elseif nargin < 3
    center = [0;0;0];
  end
  
  diameter = options.diameter;
  nPointsPerSphere = options.nPointsPerSphere;
  sphCenters = options.sphCenters + repmat(center, 1, size(options.sphCenters, 2));
  
  nSph = size(map, 1);
  D = zeros(nSph, 1);
  lcmClient = LCMGLClient('CapabilityMap');
  for sph = 1:nSph
    nDir = nnz(map(sph,:));
    D(sph) = nDir/nPointsPerSphere;
    if D(sph) > 0  && sphCenters(1,sph) > 0
      color = hsv2rgb([D(sph) 1 1]);
      lcmClient.glColor3f(color(1), color(2), color(3))
      lcmClient.sphere(sphCenters(:,sph), diameter/2, 20, 20);      
    end
  end
  lcmClient.switchBuffers();
end