function [xtraj, info, simVars, statVars] = exploringRRT(options, rng_seed)
  
  if nargin < 1 || isempty(options), options = struct(); end
  
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
  if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
  if ~isfield(options,'planning_mode'), options.planning_mode = 'multiRRT'; end;
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'scene'), options.scene = 4; end;
  if ~isfield(options,'model'), options.model = 'val2'; end;
  if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
  if ~isfield(options,'graspingHand'), options.graspingHand = 'right'; end;
  if ~isfield(options,'costType'), options.costType = 'length'; end;
  if ~isfield(options,'firstFeasibleTraj'), options.firstFeasibleTraj = false; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  if ~isfield(options,'nTrees'), options.nTrees = 4; end;
  if ~isfield(options,'feet_constraint'), options.feet_constraint = 'sliding'; end
  
  if ~any(strcmp(options.model, {'val1', 'val2'}))
    error('Only valkyrie robot is supported')
  end
  
  options.floating = true;
  options.joint_v_max = 15*pi/180;
  
  if isempty(options.robot)
    r = Scenes.generateRobot(options);
  else
    r = options.robot;
  end
  
  if options.visualize
    pose_publisher = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE', true, r.getPositionFrame.getCoordinateNames);
    plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, r.getPositionFrame.getCoordinateNames);
    Scenes.visualizeOctomap(options);
  end
  
  if nargin > 1
    rng(rng_seed);
  end
  rndSeed = rng;
  save lastRndg.mat rndSeed
  
  g_hand = Scenes.getGraspingHand(options, r);
  point_in_link_frame = Scenes.getPointInLinkFrame(options);
  
  q_nom = Scenes.getFP(options.model, r);
  
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
  Q = diag(cost);
  ikoptions = IKoptions(r);
  ikoptions = ikoptions.setMajorIterationsLimit(100);
  ikoptions = ikoptions.setQ(Q);
  ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
  
  %Set start pose constraints and compute starting configuration
  startPoseConstraints = [Scenes.generateFeetConstraints(options, r, q_nom),...
                         {Scenes.generateQuasiStaticConstraint(options, r),...
                          Scenes.nonGraspingHandDistanceConstraint(options, r, 0.4)}];
  [q_start, info] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
  if info > 10, error('Starting pose is invalid'); end
  if options.visualize
    pose_publisher.publish([q_start; zeros(size(q_start))], get_timestamp_now())
  end
  
  %Compute final pose
  
  cm = CapabilityMap([getenv('DRC_BASE') '/../drc-testing-data/final_pose_planner/val_description/capabilityMap.mat']);
  x_end.val1.right = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 pi/2])];
  x_end.val1.left = [Scenes.getTargetObjPos(options)'; rpy2quat([0 0 -pi/2])];
  x_end.val2 = x_end.val1;

  finalPose = FinalPosePlanner(r, g_hand, q_start, x_end.(options.model).(options.graspingHand), ...
    startPoseConstraints, q_nom, cm, ikoptions, ...
    'graspinghand', options.graspingHand, ...
    'endeffectorpoint', point_in_link_frame, ...
    'debug', false);
  
  [xGoalFull, info] = finalPose.findFinalPose(Scenes.getOctomap(options));
  if info > 10, error('Failed to find a final pose'); end
  q_end = xGoalFull(8:end);
  if options.visualize
    pose_publisher.publish([q_end; zeros(size(q_end))], get_timestamp_now())
  end
  
  %Recompute starting configuration
  options.feet_constraint = 'fixed';
  startPoseConstraints = [Scenes.generateFeetConstraints(options, r, q_end),...
                         {Scenes.generateQuasiStaticConstraint(options, r),...
                          Scenes.nonGraspingHandDistanceConstraint(options, r, 0.4)}];
  [q_start, info] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
  if info > 10, error('Starting pose is invalid'); end
  if options.visualize
    pose_publisher.publish([q_start; zeros(size(q_start))], get_timestamp_now())
  end
  
  
  %Create RRTs

  kinsol = r.doKinematics(q_start);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  kinsol = r.doKinematics(q_end);
  xyz_quat_goal = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  x_start = [xyz_quat_start;q_start];
  x_goal = [xyz_quat_goal;q_end];
  xyz_box_edge_length = 2;
  
  if ~strcmp(options.planning_mode, 'multiRRT')
    %{
    min_distance = 0.01;
    active_collision_options.body_idx = setdiff(1:r.getNumBodies(),inactive_collision_bodies);
    options.display_after_every = 1;
    TA = TaskSpaceMotionPlanningTree(r, g_hand, point_in_link_frame);
    TA  = TA.setMinDistance(min_distance);
    TA  = TA.setOrientationWeight(1);
    TA.max_edge_length = 0.05;
    TA.max_length_between_constraint_checks = TA.max_edge_length;
    TA.angle_tol = 10*pi/180;
    TA.position_tol = 1e-3;
    TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), inactive_collision_bodies);
    TA.trees{TA.cspace_idx}.ikoptions = ikoptions;
    xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
    xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;

    %Naive reduction of the task space
    xyz_min = [x_start(1) - 0.2; min([x_start(2), x_goal(2)]) - 0.5; 0];
    xyz_max = [x_goal(1) + 0.2; max([x_start(2), x_goal(2)]) + 0.5; xyz_max(3)];

    TA = TA.setTranslationSamplingBounds(xyz_min, xyz_max);
    TA = TA.addKinematicConstraint(startPoseConstraints{:});
    TA = TA.setNominalConfiguration(q_nom);

    TA = TA.compile();
    TB = TA;
    TC = TA;
    TD = TA;
    assert(TA.checkConstraints(x_start))
    assert(TB.checkConstraints(x_goal))
  
    TA = TA.setLCMGL('TA',[1,0,0]);
    TB = TB.setLCMGL('TB',[0,0,1]);
    %}
  end
  
  qNomCFile.val1.right = 'valkyrie/valkyrie_fp_rHand_up';
  qNomDFile.val1.right = 'valkyrie/valkyrie_fp_rHand_up_right';
  qNomCFile.val2.right = 'val_description/valkyrie2_fp_rHand_up';
  qNomCFile.val2.left = 'val_description/valkyrie2_fp_lHand_up';
  qNomDFile.val2.right = 'val_description/valkyrie2_fp_rHand_up_right';
  qNomDFile.val2.left = 'val_description/valkyrie2_fp_lHand_up_left';
  qNomCFile.v5.right = 'atlas_v5/atlasv5_fp_rHand_up';
  qNomCFile.v5.left = 'atlas_v5/atlasv5_fp_lHand_up';
  qNomDFile.v5.right = 'atlas_v5/atlasv5_fp_rHand_up_right';
  qNomDFile.v5.left = 'atlas_v5/atlasv5_fp_lHand_up_left';
  
  qNominalC = Scenes.getFP(qNomCFile.(options.model).(options.graspingHand), r);
  qNominalD = Scenes.getFP(qNomDFile.(options.model).(options.graspingHand), r);

  kinsol = r.doKinematics(qNominalC);
  EEpose = r.forwardKin(kinsol, Scenes.getGraspingHand(options, r), Scenes.getPointInLinkFrame(options), 2);
  constraints = [startPoseConstraints, Scenes.generateEEConstraints(r, options, EEpose)];
  qStartC = inverseKin(r, qNominalC, qNominalC, constraints{:}, ikoptions);
  if options.visualize
    pose_publisher.publish([qStartC; zeros(size(qStartC))], get_timestamp_now())
  end
  kinsol = r.doKinematics(qStartC);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  xStartC = [xyz_quat_start; qStartC];
   
  kinsol = r.doKinematics(qNominalD);
  EEpose = r.forwardKin(kinsol, Scenes.getGraspingHand(options, r), Scenes.getPointInLinkFrame(options), 2);
  constraints = [startPoseConstraints, Scenes.generateEEConstraints(r, options, EEpose)];
  qStartD = inverseKin(r, qNominalD, qNominalD, constraints{:}, ikoptions);
  if options.visualize
    pose_publisher.publish([qStartD; zeros(size(qStartD))], get_timestamp_now())
  end
  kinsol = r.doKinematics(qStartD);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  xStartD = [xyz_quat_start; qStartD];
  
  if options.visualize
    pose_publisher.publish([q_start; zeros(size(q_start))], get_timestamp_now())
  end
  
  rrt_timer = tic;
  display('Computing motion plan ...')
  switch options.planning_mode
%     case 'rrt'
%       [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, TA, options);
%     case 'rrt_connect'
%       [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
    case 'multiRRT'
      
      optionsPlanner = struct();
      if ~isfield(optionsPlanner,'costType'), optionsPlanner.costType = 'length'; end;
      
      switch options.nTrees
        case 4
          additionalTrees = [xStartC, xStartD];
        case 3
          additionalTrees = xStartC;
        case 2
          additionalTrees = [];
      end
          multiTree = MultipleTreePlanner(r, g_hand, x_start, x_end.(options.model).(options.graspingHand), ...
            additionalTrees, startPoseConstraints, q_nom, Scenes.getOctomap(options), ...
            'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);

      [multiTree, info, cost, q_path] = multiTree.rrtStar(optionsPlanner, xGoalFull);

      if info == 1
        disp('Motion plan computed')
        path_length = size(q_path,2);
        plan_publisher.publish([ones(1, path_length); zeros(1, path_length);q_path(8:end,:);zeros(r.num_positions, path_length)], linspace(0,1,path_length), now() * 24 * 60 * 60, ones(1, path_length))
        xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(r.getNumVelocities(), size(q_path,2))] ));
      else
        disp('Failded to compute a motion plan')
        xtraj = [];
        info = 13;
      end
      if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else qtraj = []; end;
%       if ~isempty(qtraj), publishTraj(r, plan_publisher, [q_path(8:end,:);zeros(size(q_path(8:end,:)))]); end


      TA = multiTree.trees(1);
  end
  rrt_time = toc(rrt_timer);  
  
  if (info == Info.SUCCESS)
    %{
    if ~any(strcmp(options.planning_mode, {'rrt*', 'multiRRT'}))
      T_smooth = TA;
      simVars.TConnected = T_smooth;
      path_ids_C = path_ids_A;
      % interp_weight determines how much consideration is given to joint
      % space distance during smoothing:
      %  * 0 - end-effector distance only
      %  * 1 - joint-space distance only
      T_smooth.interp_weight = 0.5;
      q_idx = TA.idx{TA.cspace_idx};
      if (options.n_smoothing_passes > 0)
        smoothing_timer = tic;
        T_smooth = T_smooth.setLCMGL('T_smooth', TA.line_color);
        [T_smooth, id_last] = T_smooth.recursiveConnectSmoothing(path_ids_A, options.n_smoothing_passes, options.visualize);
        path_ids_A = T_smooth.getPathToVertex(id_last);
        smoothing_time = toc(smoothing_timer);
        if options.visualize
          drawTree(TA);
          drawTree(TB);
          drawPath(T_smooth, path_ids_A);
        end
      end
      q_path = extractPath(T_smooth, path_ids_A);
    end
    %}
    path_length = size(q_path,2);
    
    % Scale timing to obey joint velocity limits
    % Create initial spline
    q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(TA.idx{TA.cspace_idx},:)));
    t = linspace(0, 1, 10*path_length);
    q_path = eval(q_traj, t);
    
    % Determine max joint velocity at midpoint of  each segment
    t_mid = mean([t(2:end); t(1:end-1)],1);
    v_mid = max(abs(q_traj.fnder().eval(t_mid)), [], 1);
    
    % Adjust durations to keep velocity below max
    t_scaled = [0, cumsum(diff(t).*v_mid/mean(options.joint_v_max))];
    tf = t_scaled(end);
    
    % Warp time to give gradual acceleration/deceleration
    t_scaled = tf*(-real(acos(2*t_scaled/tf-1)+pi)/2);
    [t_scaled, idx_unique] = unique(t_scaled,'stable');
    
    rState = r.getStateFrame();
    xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
    xtraj = xtraj.setOutputFrame(r.getStateFrame());
    
    %Output structures
    switch options.planning_mode
      case 'rrt_connect'
        simVars.TA = TA;
        simVars.TB = TB;
        simVars.T_smooth = T_smooth;
        simVars.path_ids_A = path_ids_A;
        simVars.path_ids_C = path_ids_C;
        simVars.rndSeed = rndSeed;
        statVars.rrtTime = rrt_time;
        statVars.smoothingTime = smoothing_time;
        statVars.totTime = rrt_time + smoothing_time;
        statVars.TAn = TA.n;
        statVars.TBn = TB.n;
        statVars.TCn = simVars.TConnected.n;
        statVars.TSn = T_smooth.n;
        if info == 1
          statVars.info = Info(Info.SUCCESS);
        else
          statVars.info = Info(Info.FAIL_OTHER);
        end
        q_path = extractPath(T_smooth, path_ids_A);
        TSlengths = q_path(1:3,2:end)-q_path(1:3, 1:end-1);
        statVars.TSlength = sum(diag(sqrt(TSlengths'*TSlengths)));
        q_pathC = extractPath(simVars.TConnected, path_ids_C);
        TClengths = q_pathC(1:3,2:end)-q_pathC(1:3, 1:end-1);
        statVars.TClength = sum(diag(sqrt(TClengths'*TClengths)));
        statVars.options = rmfield(options, {'robot', 'terrain'});
      case 'multiRRT'
        if size(x_end.(options.model).(options.graspingHand), 2) <= 7
%           statVars.finalPoseTime = info.finalPoseTime;
%           statVars.finalPoseCost = info.finalPoseCost;
        else
          statVars.finalPoseTime = 0;
          statVars.finalPoseCost = 0;
        end
%         statVars.reachingTime = info.reachingTime;
%         statVars.improvingTime = info.improvingTime;
%         statVars.shortcutTime = info.shortcutTime;
%         statVars.rebuildTime = info.rebuildTime;
%         statVars.reachingNpoints = info.reachingNpoints;
%         statVars.improvingNpoints = info.improvingNpoints;
%         statVars.shortcutNpoints = info.shortcutNpoints;
%         statVars.rebuildNpoints = info.rebuildNpoints;
%         statVars.costReaching = info.costReaching;
%         statVars.costImproving = info.costImproving;
%         statVars.costShortcut = info.costShortcut;
%         statVars.nPoints = info.nPoints;
%         statVars.nTrees = info.nTrees;
        statVars.rndSeed = rndSeed; 
        statVars.options = rmfield(options, 'robot');
        simVars.info = info;
%         if options.visualize
%           fprintf(['TIMING:\n',...
%                   '\tFinalPoseTime: %.2f\n',...
%                   '\tReaching Time: %.2f\n',...
%                   '\tImproving Time: %.2f\n',...
%                   '\tShortcut Time: %.2f\n',...
%                   '\trebuild Time: %.2f\n',...
%                   '\ttotal Time: %.2f\n'],...
%                   info.finalPoseTime, info.reachingTime, info.improvingTime,...
%                   info.shortcutTime, info.rebuildTime, rrt_time);
%         end
    end
    
    %if options.visualize      
    %  s.publishTraj(q_traj, 1);
    %end
  else
    xtraj = [];
    v = [];
    statVars.finalPoseTime = [];
    statVars.reachingTime = [];
    statVars.improvingTime = [];
    statVars.shortcutTime = [];
    statVars.rebuildTime = [];
    statVars.reachingNpoints = [];
    statVars.improvingNpoints = [];
    statVars.shortcutNpoints = [];
    statVars.rebuildNpoints = [];
    statVars.collisionFinalPoseTime = [];
    statVars.collisionImprovingTime = [];
    statVars.collisionReachingTime = [];
    statVars.collisionShortcutTime = [];
    statVars.IKFinalPoseTime = [];
    statVars.IKImprovingTime = [];
    statVars.IKReachingTime = [];
    statVars.IKRebuildTime = [];
    statVars.IKShortcutTime = [];
    statVars.collisionTime = [];
    statVars.IKTime = [];
    statVars.costReaching = [];
    statVars.costImproving = [];
    statVars.costShortcut = [];
    statVars.finalPoseCost = [];
    statVars.nPoints = [];
    statVars.rndSeed = rndSeed; 
    statVars.options = options;
    simVars.info = info;
    fprintf('Failed to find a solution (%s)\n', info)
  end
  
end
  
  function publishTraj(r, plan_publisher, xtraj)
    if ~isa(xtraj, 'PPTrajectory')
      xtraj = PPTrajectory(xtraj);
    end
    utime = now() * 24 * 60 * 60;
    nq_atlas = r.getNumPositions;
    ts = xtraj.pp.breaks;
    q = xtraj.eval(ts);
    xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
    xtraj_atlas(2+(1:nq_atlas),:) = q(1:nq_atlas,:);
    snopt_info_vector = 1*ones(1, size(xtraj_atlas,2));
    plan_publisher.publish(xtraj_atlas, ts, utime, snopt_info_vector);
  end



