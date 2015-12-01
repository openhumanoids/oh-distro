function [info, debug_vars] = exploringFPP(options, rng_seed)
  
  if nargin < 1 || isempty(options), options = struct(); end
  
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  warning('off','Drake:CollisionFilterGroup:DiscardingCollisionFilteringInfo');
  warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'scene'), options.scene = 1; end;
  if ~isfield(options,'model'), options.model = 'val2'; end;
  if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
  if ~isfield(options,'graspingHand'), options.graspingHand = 'right'; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  if ~isfield(options,'back_constraint'), options.back_constraint = 'free'; end
  if ~isfield(options,'base_constraint'), options.base_constraint = 'free'; end
  if ~isfield(options,'feet_constraint'), options.feet_constraint = 'sliding'; end
  
  options.floating = true;
  
  if isempty(options.robot)
    r = Scenes.generateRobot(options);
  else
    r = options.robot;
  end
  
  if options.visualize
    pose_publisher = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE', true, r.getPositionFrame.getCoordinateNames);
    visWorld = RigidBodyManipulator();
    for b = 1:numel(r.body(1).visual_geometry)
      visWorld = addGeometryToBody(visWorld, 1, r.body(1).visual_geometry{b});
    end
    visWorld = visWorld.compile();
    visWorld.constructVisualizer();
    Scenes.visualizePointCloud(options);
  end
  
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
    pose_publisher.publish([q_nom; zeros(size(q_nom))], get_timestamp_now())
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
  cost(13:26) = cost(13:26)/2;
  Q = diag(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  
  %Set start pose constraints and compute starting configuration
  startPoseConstraints = [Scenes.generateFeetConstraints(options, r, q_nom),...
                         {Scenes.generateQuasiStaticConstraint(options, r),...
                          Scenes.generateBackConstraint(options, r, q_nom), ...
                          Scenes.generateBaseConstraint(options, r, q_nom), ...
                          Scenes.nonGraspingHandDistanceConstraint(options, r, 0.4)}];
  [q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
  
  if options.visualize
    pose_publisher.publish([q_start; zeros(size(q_start))], get_timestamp_now())
    %     drawLinkFrame(r, g_hand, q_start, 'Grasping Hand Start');
    %     drawLinkFrame(r, Scenes.getNonGraspingHand(options, r), q_start, 'Non Grasping Hand Start');
    %     drawLinkFrame(r, r.findLinkId('l_ufarm'), q_start, 'Forearm Start');
  end
  
  cm = CapabilityMap([getenv('DRC_BASE') '/software/control/matlab/data/val_description/capabilityMap.mat']);
  x_end = Scenes.getDesiredEePose(options);

  finalPose = FinalPoseProblem(r, g_hand, q_start, x_end, ...
    startPoseConstraints, q_nom, cm, ikoptions, ...
    'graspinghand', options.graspingHand, ...
    'endeffectorpoint', point_in_link_frame, ...
    'debug', true);

  [xGoalFull, info, debug_vars] = finalPose.findFinalPose(Scenes.getPointCloud(options));
  q_end = xGoalFull(8:end);
  if options.visualize && info == 1
    pose_publisher.publish([q_end; zeros(size(q_end))], get_timestamp_now())
  end
end