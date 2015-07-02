function [map, options] = capabilityMap(options)
  if nargin < 1 || isempty(options), options = struct(); end
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'robot'), options.robot = []; end;  
  
  % PARAMETERS  
  if ~isfield(options,'diameter'), options.diameter = 0.05; end;
  if ~isfield(options,'nPointsPerSphere'), options.nPointsPerSphere = 50; end;
  if ~isfield(options,'nSamples'), options.nSamples = 10; end;
  if ~isfield(options,'nOrient'), options.nOrient = 22; end;
  if ~isfield(options,'angTolerance'), options.angTolerance = 1*pi/180; end;
  if ~isfield(options,'posTolerance'), options.posTolerance = 0.01; end;
  diameter = options.diameter;
  nPointsPerSphere = options.nPointsPerSphere;
  nSamples = options.nSamples;
  nOrient = options.nOrient;
  angTolerance = options.angTolerance;
  posTolerance = options.posTolerance;
  
  options.floating = false;
  options.rotation_type = 2;
  
  %Use complete model to compute arm length
  
  if isempty(options.robot)
    urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
    r = RigidBodyManipulator(urdf,options);
    options.robot = r;
  else
    r = options.robot;
  end
  
  q = zeros(r.num_positions, 1);
  kinsol = r.doKinematics(q, [], options);
  
  shoulder = r.findLinkId('RightShoulderAdductor');
  palm = r.findLinkId('RightPalm');
  shoulderPos = r.forwardKin(kinsol, shoulder, [0;0;0], options);
  palmPos = r.forwardKin(kinsol, palm, [0;0;0], options);
  distance = sqrt(sum((shoulderPos-palmPos).^2));
  
  %Use an arm-only robot for the computation  
  
  urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_right_arm_only.urdf');
  r = RigidBodyManipulator(urdf,options);
  if options.visualize
    v = r.constructVisualizer();
  end
  q = zeros(r.num_positions, 1);
  kinsol = r.doKinematics(q, [], options);
  palm = r.findLinkId('RightPalm');
  shoulder = r.findLinkId('RightShoulderAdductor');
  shoulderPose = r.forwardKin(kinsol, shoulder, [0;0;0], options);
%   drawTreePoints(shoulderPose, 'frame', true);
  
  % Workspace discretization
  nSphPerEdge = 2*ceil(distance/diameter);
  boxEdge = nSphPerEdge * diameter;
  nSph = nSphPerEdge^3;
  sphX = linspace(-(boxEdge-diameter)/2, (boxEdge-diameter)/2, nSphPerEdge);
  sphY = linspace(-(boxEdge-diameter)/2, (boxEdge-diameter)/2, nSphPerEdge);
  sphZ = linspace(-(boxEdge-diameter)/2, (boxEdge-diameter)/2, nSphPerEdge);
  [vecY, vecX, vecZ] = meshgrid(sphY, sphX, sphZ);
  vecX = reshape(vecX, numel(vecX), 1);
  vecY = reshape(vecY, numel(vecY), 1);
  vecZ = reshape(vecZ, numel(vecZ), 1);
  sphCenters = [vecX vecY vecZ]';
  options.sphCenters = sphCenters;
  
  [P, frames] = distributePointsOnSphere(nPointsPerSphere);
  map = false(nSph, nPointsPerSphere);
  
%   if options.visualize
%     lcmClient = LCMGLClient('Spheres');
%     lcmClient.glColor4f(1, 0, 0, 0.05)
%     for center = 1:size(sphCenters,2)
%       lcmClient.sphere(sphCenters(:,center), diameter/2, 20, 20);
%     end
%     lcmClient.switchBuffers();
%   end
  
  %IK Options
  Q = diag([6 5 4 3 2 1]);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  
  options.rotation_type = 0;
  
  global IKTimes
  for sample = 1:nSamples
    fprintf('Sample %d of %d\n', sample, nSamples)
    q = r.joint_limit_min + (r.joint_limit_max-r.joint_limit_min).*rand(r.num_positions,1);
    kinsol = r.doKinematics(q);
    pos = r.forwardKin(kinsol, palm, [0;0;0], options);
    sub = ceil(pos/diameter) + (nSphPerEdge/2) * ones(3,1);
    sphInd = sub2ind(nSphPerEdge * ones(1,3), sub(1), sub(2), sub(3));
    if options.visualize
      drawTreePoints(sphCenters(:,sphInd), 'pointsize', diameter/2)
      v.draw(0, q);
    end
    for point = 1:nPointsPerSphere
      pos = sphCenters(:,sphInd) + P(:, point)*diameter/2;
      quat = rotmat2quat(squeeze(frames(point,:,:)));
      posConstraint = WorldPositionConstraint(r, palm, [0;0;0], pos - posTolerance/2, pos + posTolerance/2);
      GazeConstraint = WorldGazeDirConstraint(r, palm, [-1; 0; 0], P(:, point), angTolerance);
      [qNew, info, infeasibleConstraint] = r.inverseKin(q, q, posConstraint, GazeConstraint, ikoptions);
      if options.visualize
        drawTreePoints([pos; quat], 'frame', true, 'text', 'frame')
        v.draw(0, qNew)
        drawLinkFrame(r, palm, qNew, 'palm');
      end
      if info < 13
        map(sphInd, point) = true;
      end
    end
  end
end