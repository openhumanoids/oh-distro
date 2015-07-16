function drawCapabilityMap(mapFile, center)
  load(mapFile);
  
  if ~isfield(options, 'drawRobot'), options.drawRobot = false; end
  
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
  elseif nargin < 2
    center = [0;0;0];
  end
  
  gui = capabilityMapControls(0, -pi/6, 0.8, 1.5, 5.5, mapFile, center);
  
  [map, reachabilityIndex, sphCenters] = pruneCapabilityMap(mapFile, 0, -pi/6, 0.8, 1.5, 5.5);
  sphCentersShoulder = sphCenters + repmat(center, 1, size(sphCenters, 2));
  diameter = options.sphDiameter;
  
  nSph = size(map, 1);
  lcmClient = LCMGLClient('CapabilityMap');
  for sph = 1:nSph
    color = hsv2rgb([reachabilityIndex(sph) 1 1]);
    lcmClient.glColor3f(color(1), color(2), color(3))
    lcmClient.sphere(sphCentersShoulder(:,sph), diameter/2, 20, 20);   
  end
  lcmClient.switchBuffers();
end