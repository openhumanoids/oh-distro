function info = exploringFPP(options, rng_seed)
  
  if nargin < 1 || isempty(options), options = struct(); end
  
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'scene'), options.scene = 1; end;
  if ~isfield(options,'model'), options.model = 'val2'; end;
  if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
  if ~isfield(options,'graspingHand'), options.graspingHand = 'left'; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  
  
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain();
  options.joint_v_max = 15*pi/180;
  
  if isempty(options.robot)
    r = Scenes.generateScene(options);
  else
    r = options.robot;
  end
  
  v = r.constructVisualizer();
  
%   needed for IKServer
%   addpath(fullfile(getDrakePath(), '../../', 'ddapp/src/matlab'))
%   fixed_point_file = [getDrakePath(), '/../../control/matlab/data/val_description/valkyrie_fp_june2015.mat'];
%   left_foot_link = 'LeftFoot';
%   right_foot_link = 'RightFoot';
%   runIKServer
% 
%   joint_names = {r.getPositionFrame.getCoordinateNames{1:getNumPositions(r)}}';
%   pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
  
  if nargin > 1
    rng(rng_seed);
  end
  rndSeed = rng;
  save lastRndg.mat rndSeed
  
  lFoot = Scenes.getLeftFoot(options, r);
  rFoot = Scenes.getRightFoot(options, r);
  
  g_hand = Scenes.getGraspingHand(options, r);
  point_in_link_frame = Scenes.getPointInLinkFrame(options);
  
  q_nom = Scenes.getFP(options.model, r);
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if options.visualize
    v.draw(0, q_nom)
  end
  
  %Set IK options
  ik_seed_pose = q_nom;
  ik_nominal_pose = q_nom;
  cost = Point(r.getPositionFrame(),10);
  for i = r.getNumBodies():-1:1
    if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
      cost(r.getBody(r.getBody(i).parent).position_num) = ...
        cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
    end
  end
  cost(1:6) = max(cost(7:end))/2;
  cost = cost/min(cost);
  Q = diag(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  
  %Set start pose constraints and compute starting configuration
  startPoseConstraints = [Scenes.fixedFeetConstraints(options, r),...
                          {Scenes.addQuasiStaticConstraint(options, r),...
                          Scenes.nonGraspingHandDistanceConstraint(options, r, 0.4)}];
  [q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
  
  if options.visualize
    v.draw(0, q_start)
    %     drawLinkFrame(r, g_hand, q_start, 'Grasping Hand Start');
    %     drawLinkFrame(r, Scenes.getNonGraspingHand(options, r), q_start, 'Non Grasping Hand Start');
    %     drawLinkFrame(r, r.findLinkId('l_ufarm'), q_start, 'Forearm Start');
  end
  
  cm = CapabilityMap([getenv('DRC_BASE') '/software/control/matlab/data/val_description/capabilityMap.mat']);
  x_end.val1.right = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 pi/2])];
  x_end.val1.left = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 -pi/2])];
  x_end.val2 = x_end.val1;

  finalPose = FinalPoseProblem(r, g_hand, q_start, x_end.(options.model).(options.graspingHand), ...
    startPoseConstraints, q_nom, ...
    'capabilityMap', cm,...
    'graspinghand', options.graspingHand, ...
    'ikoptions', ikoptions, ...
    'endeffectorpoint', point_in_link_frame);

  [xGoalFull, info] = finalPose.findFinalPose();
  if options.visualize
    v.draw(0, xGoalFull(8:end))
  end
end