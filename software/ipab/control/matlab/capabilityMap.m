function [map, options] = capabilityMap(options)
  
  if nargin < 1 || isempty(options), options = struct(); end
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'visualize'), options.visualize = false; end;
  if ~isfield(options,'robot'), options.robot = []; end;  
  
  if ~isfield(options,'diameter'), options.diameter = 0.1; end; % 0.05;
  if ~isfield(options,'nPointsPerSphere'), options.nPointsPerSphere = 50; end;
  if ~isfield(options,'nSamples'), options.nSamples = 1000; end;
  if ~isfield(options,'nOrient'), options.nOrient = 5; end;
  if ~isfield(options,'angTolerance'), options.angTolerance = 1*pi/180; end;
  if ~isfield(options,'posTolerance'), options.posTolerance = 0.01; end;
  
  options.floating = true;
  options.use_mex = false;
  options.rotation_type = 0;
  
  if isempty(options.robot)
    urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
    r = RigidBodyManipulator(urdf,options);
    options.robot = r;
  else
    r = options.robot;
  end
%   v = r.constructVisualizer();
  
  q = zeros(r.num_positions, 1);
  kinsol = r.doKinematics(q, [], options);
  
  shoulder = r.findLinkId('RightShoulderAdductor');
  palm = r.findLinkId('RightPalm');
  wrist = r.findLinkId('RightWristYoke');
  %   trunk = r.findLinkId('Trunk');
  %   world = r.findLinkId('world');
  
  dof  = [];
  parent = wrist;
  while parent ~= r.getBody(shoulder).parent
    parent = r.getBody(parent).parent;
    joint = r.getBody(parent).position_num;
    dof(end +1) = joint;
  end
  nDof = length(dof);
  shoulderT = cell2mat(kinsol.T(shoulder));
  palmT = cell2mat(kinsol.T(palm));
  distance = sqrt(sum((shoulderT(1:3,4)-palmT(1:3,4)).^2));
  
  % PARAMETERS
  diameter = options.diameter;
  nPointsPerSphere = options.nPointsPerSphere;
  nSamples = options.nSamples;
  nOrient = options.nOrient;
  angTolerance = options.angTolerance;
  posTolerance = options.posTolerance;
  
  boxCenter = shoulderT(1:3,4);
  nSphPerEdge = 2*ceil(distance/diameter);
  boxEdge = nSphPerEdge * diameter;
  nSph = nSphPerEdge^3;
  sphX = linspace(boxCenter(1)-(boxEdge-diameter)/2, boxCenter(1)+(boxEdge-diameter)/2, nSphPerEdge);
  sphY = linspace(boxCenter(2)-(boxEdge-diameter)/2, boxCenter(2)+(boxEdge-diameter)/2, nSphPerEdge);
  sphZ = linspace(boxCenter(3)-(boxEdge-diameter)/2, boxCenter(3)+(boxEdge-diameter)/2, nSphPerEdge);
  [vecY, vecX, vecZ] = meshgrid(sphY, sphX, sphZ);
  vecX = reshape(vecX, numel(vecX), 1);
  vecY = reshape(vecY, numel(vecY), 1);
  vecZ = reshape(vecZ, numel(vecZ), 1);
  sphCenters = [vecX vecY vecZ]';
  options.sphCenters = sphCenters;
  
  if options.visualize
    lcmClient = LCMGLClient('Spheres');
    lcmClient.glColor4f(1, 0, 0, 0.05)
    for center = 1:size(sphCenters,2)
      lcmClient.sphere(sphCenters(:,center), diameter/2, 20, 20);
    end
    lcmClient.switchBuffers();
  end
  
  [P, frames] = distributePointsOnSphere(nPointsPerSphere);
  angle = 2*pi/nOrient;
  rotMat = [1 0           0;...
            0 cos(angle) -sin(angle);...
            0 sin(angle)  cos(angle)];
  xRot = [1 0 0; 0 0 -1; 0 1 0];
  map = false(nSph, nPointsPerSphere);
  
  %IK Options
  cost = Point(r.getPositionFrame(),10);
  for i = r.getNumBodies():-1:1
    if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
      cost(r.getBody(r.getBody(i).parent).position_num) = ...
        cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
    end
  end
  cost = cost/min(cost);
  Q = diag(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  postureConstraint = PostureConstraint(r, [-inf, inf]);
  fixedJoints = setdiff(1:r.num_positions, dof);
  lb = min([max([r.joint_limit_min(fixedJoints) zeros(r.num_positions - nDof, 1)], [], 2), r.joint_limit_max(fixedJoints)], [], 2);
  ub = max([min([r.joint_limit_max(fixedJoints) zeros(r.num_positions - nDof, 1)], [], 2), r.joint_limit_min(fixedJoints)], [], 2);
  postureConstraint = postureConstraint.setJointLimits(fixedJoints', lb, ub);
  
  for sample = 1:nSamples
    fprintf('Sample %d of %d\n', sample, nSamples)
    q(dof) = r.joint_limit_min(dof) + (r.joint_limit_max(dof)-r.joint_limit_min(dof)).*rand(nDof,1);
%     v.draw(0, q);
    kinsol = r.doKinematics(q);
    pos = r.forwardKin(kinsol, palm, [0;0;0], options) - boxCenter;
    sub = ceil(pos/diameter) + (nSphPerEdge/2) * ones(3,1);
    sphInd = sub2ind(nSphPerEdge * ones(1,3), sub(1), sub(2), sub(3));
%     drawTreePoints(sphCenters(:,sphInd), 'pointsize', diameter/2 + 0.01)
    for point = 1:nPointsPerSphere
      frame = squeeze(frames(point,:,:)) * xRot;
      quat = rotmat2quat(frame);
      pos = sphCenters(:,sphInd) + P(:, point)*diameter/2;
%       drawTreePoints([pos; quat], 'frame', true, 'text', 'frame')
      posConstraint = WorldPositionConstraint(r, wrist, [0; 0; 0], pos - posTolerance/2, pos + posTolerance/2);
      quatConstraint = WorldQuatConstraint(r, wrist, quat, angTolerance);
      [qNew, info, infeasibleConstraint] = r.inverseKin(q, q, postureConstraint, posConstraint, quatConstraint, ikoptions);
      
%       v.draw(0, qNew)
      orient = 0;
      while info >= 13 && orient < nOrient
        frame = frame * rotMat;
        quat = rotmat2quat(frame);
%         drawTreePoints([pos; quat], 'frame', true, 'text', 'frame')
        quatConstraint = WorldQuatConstraint(r, wrist, quat, angTolerance);
        [qNew, info, infeasibleConstraint] = r.inverseKin(q, q, postureConstraint, posConstraint, quatConstraint, ikoptions);
%         v.draw(0, qNew)
        orient = orient + 1;
      end
      if info < 13
        map(sphInd, point) = true;
      end
    end
  end
end