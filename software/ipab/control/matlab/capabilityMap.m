function options = capabilityMap(options)
  
  
  if nargin < 1 || isempty(options), options = struct(); end
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  
  options.floating = true;
  options.use_mex = false;
  options.rotation_type = 0;
  %options.terrain = MyRigidBodyFlatTerrain();
  options.joint_v_max = 15*pi/180;
  if isempty(options.robot)
    urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
    r = RigidBodyManipulator(urdf,options);
    options.robot = r;
  else
    r = options.robot;
  end
  v = r.constructVisualizer();
  
  q = zeros(r.num_positions, 1);
  kinsol = r.doKinematics(q, [], options);
  
  shoulder = r.findLinkId('RightShoulderAdductor');
  palm = r.findLinkId('RightPalm');
  forearm = r.findLinkId('RightForearm');
  trunk = r.findLinkId('Trunk');
  world = r.findLinkId('world');
  
  dof  = [];
  parent = forearm;
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
  diameter = 0.05;
  nPointsPerSphere = 200;
  nSamples = 100;
  nOrient = 10;
  
  boxCenter = shoulderT(1:3,4);
  nSphPerEdge = 2*ceil(distance/diameter);
  boxEdge = nSphPerEdge * diameter;
  nSph = nSphPerEdge^3;
  sphX = linspace(boxCenter(1)-(boxEdge-diameter)/2, boxCenter(1)+(boxEdge-diameter)/2, nSphPerEdge);
  sphY = linspace(boxCenter(2)-(boxEdge-diameter)/2, boxCenter(2)+(boxEdge-diameter)/2, nSphPerEdge);
  sphZ = linspace(boxCenter(3)-(boxEdge-diameter)/2, boxCenter(3)+(boxEdge-diameter)/2, nSphPerEdge);
  [vecX, vecY, vecZ] = meshgrid(sphX, sphY, sphZ);
  vecX = reshape(vecX, numel(vecX), 1);
  vecY = reshape(vecY, numel(vecY), 1);
  vecZ = reshape(vecZ, numel(vecZ), 1);
  sphCenters = [vecX vecY vecZ]';  
  
  if options.visualize
    lcmClient = LCMGLClient('Spheres');
    lcmClient.glColor4f(1, 0, 0, 0.05)
    for i = 1:size(sphCenters,2)
      lcmClient.sphere(sphCenters(:,i), diameter/2, 20, 20);
    end
    lcmClient.switchBuffers();
  end
  
  for i = 1:nSamples    
     q(dof) = r.joint_limit_min(dof) + (r.joint_limit_max(dof)-r.joint_limit_min(dof)).*rand(nDof,1);
     v.draw(0, q);
     kinsol = r.doKinematics(q);
     pos = r.forwardKin(kinsol, palm, [0;0;0], options) - boxCenter;
     sphNum = ceil(pos/diameter) + (nSphPerEdge/2) * ones(3,1);
     drawTreePoints([sphX(sphNum(1)); sphY(sphNum(2)); sphZ(sphNum(3))], 'pointsize', diameter/2 + 0.01)       
  end
  
  P = distributePointsOnSphere(nPointsPerSphere);
  
  
  
end