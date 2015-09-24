function [xtraj, info, simVars, statVars] = exploringRRT(options, rng_seed)
  
  if nargin < 1 || isempty(options), options = struct(); end
  
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
  if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
  if ~isfield(options,'planning_mode'), options.planning_mode = 'multiRRT'; end;
  if ~isfield(options,'visualize'), options.visualize = true; end;
  if ~isfield(options,'scene'), options.scene = 1; end;
  if ~isfield(options,'model'), options.model = 'val2'; end;
  if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
  if ~isfield(options,'graspingHand'), options.graspingHand = 'right'; end;
  if ~isfield(options,'costType'), options.costType = 'length'; end;
  if ~isfield(options,'firstFeasibleTraj'), options.firstFeasibleTraj = false; end;
  if ~isfield(options,'robot'), options.robot = []; end;
  if ~isfield(options,'nTrees'), options.nTrees = 4; end;
  if ~isfield(options,'goalObject'), options.goalObject = 1; end;
  
  
  options.floating = true;
  options.terrain = MyRigidBodyFlatTerrain(); %Changed to a smaller terrain to avoid visualization problem when zooming
  options.joint_v_max = 15*pi/180;
  options.viewer = 'NullVisualizer';
  
  if isempty(options.robot)
    r = Scenes.generateScene(options);
  else
    r = options.robot;
  end
  
  addpath(fullfile(getDrakePath(), '../../', 'ddapp/src/matlab') )
  fixed_point_file = [getDrakePath(), '/../../control/matlab/data/val_description/valkyrie_fp_june2015.mat'];
  left_foot_link = 'LeftFoot';
  right_foot_link = 'RightFoot';
  runRRTIKServer
  
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
    visWorld = RigidBodyManipulator();
    for b = 1:numel(r.body(1).visual_geometry)
      visWorld = addGeometryToBody(visWorld, 1, r.body(1).visual_geometry{b});
    end
    visWorld = visWorld.compile();
    visWorld.constructVisualizer();
    s.publishTraj(PPTrajectory(q_nom), 1)
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
    s.publishTraj(PPTrajectory(q_start), 1)
    %     drawLinkFrame(r, g_hand, q_start, 'Grasping Hand Start');
    %     drawLinkFrame(r, Scenes.getNonGraspingHand(options, r), q_start, 'Non Grasping Hand Start');
    %     drawLinkFrame(r, r.findLinkId('l_ufarm'), q_start, 'Forearm Start');
  end
  
  %Set end pose constraints and compute end configuration
  goalConstraints = Scenes.addGoalConstraint(options, r);
  endPoseConstraints = [startPoseConstraints, goalConstraints];
  switch options.scene
    case 2
%       if strcmp(options.model, 'val')
%         endPoseConstraints = [endPoseConstraints, {Scenes.nonGraspingHandPositionConstraint(options, r)}];
%       else
        endPoseConstraints = [endPoseConstraints, {Scenes.graspingForearmAlignConstraint(options, r)}];
%       end
    case 3
      endPoseConstraints = [endPoseConstraints, {Scenes.pelvisOffsetConstraint(options,r)}];%, Scenes.nonGraspingHandPositionConstraint(options, r)}];
  end
  [q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, endPoseConstraints{:}, ikoptions);
  if options.visualize
    s.publishTraj(PPTrajectory(q_end), 1)
    %     drawLinkFrame(r, g_hand, q_end, 'Grasping Hand End');
    %     drawLinkFrame(r, r.findLinkId('l_ufarm'), q_end, 'Forearm End');
  end
  
  %Create RRTs
  
  % internal parts removed as they created self collisions:
  if strcmp(options.model, 'val2')
    LeftHipYawLink = r.findLinkId('LeftHipYawLink');
    RightHipYawLink = r.findLinkId('RightHipYawLink');
    LowerNeckPitchLink = r.findLinkId('LowerNeckPitchLink');
    TorsoPitchLink = r.findLinkId('TorsoPitchLink');
    TorsoYawLink = r.findLinkId('TorsoYawLink');
    
    inactive_collision_bodies = [lFoot,rFoot, LowerNeckPitchLink, RightHipYawLink,...
      LeftHipYawLink, TorsoPitchLink, TorsoYawLink];
  else
    inactive_collision_bodies = [lFoot,rFoot];
  end

  kinsol = r.doKinematics(q_start);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  kinsol = r.doKinematics(q_end);
  xyz_quat_goal = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  x_start = [xyz_quat_start;q_start];
  x_goal = [xyz_quat_goal;q_end];
  xyz_box_edge_length = 2;
  
  if ~strcmp(options.planning_mode, 'multiRRT')
    min_distance = 0.01;
    active_collision_options.body_idx = setdiff(1:r.getNumBodies(),inactive_collision_bodies);
    options.display_after_every = 1;
    if any(strcmp(options.planning_mode, 'rrt*'))
      TA = OptimalMotionPlanningTree(r, g_hand, point_in_link_frame);
    else
      TA = TaskSpaceMotionPlanningTree(r, g_hand, point_in_link_frame);
    end
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
  [qStartC, info, infeasible_constraint] = inverseKin(r, qNominalC, qNominalC, constraints{:}, ikoptions);
  if options.visualize
    s.publishTraj(PPTrajectory(qStartC), 1);
  end
  kinsol = r.doKinematics(qStartC);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  xStartC = [xyz_quat_start; qStartC];
   
  kinsol = r.doKinematics(qNominalD);
  EEpose = r.forwardKin(kinsol, Scenes.getGraspingHand(options, r), Scenes.getPointInLinkFrame(options), 2);
  constraints = [startPoseConstraints, Scenes.generateEEConstraints(r, options, EEpose)];
  [qStartD, info, infeasible_constraint] = inverseKin(r, qNominalD, qNominalD, constraints{:}, ikoptions);
  if options.visualize
    s.publishTraj(PPTrajectory(qStartD), 1);
  end
  kinsol = r.doKinematics(qStartD);
  xyz_quat_start = r.forwardKin(kinsol,g_hand,point_in_link_frame,2);
  xStartD = [xyz_quat_start; qStartD];
  
  if options.visualize
    s.publishTraj(PPTrajectory(q_start), 1);
  end
  
  rrt_timer = tic;
  %display('About to plan ...')
  switch options.planning_mode
    case 'rrt'
      [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, TA, options);
    case 'rrt_connect'
      %display('Running RRTConnect')
      [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
    case 'rrt*'
      [TA, info, cost, q_path] = TA.rrtStar(x_start, x_goal, options);
    case 'multiRRT'
      cm = CapabilityMap([fileparts(which('exploringRRT')) '/CapabilityMap/capabilityMap.mat']);
      x_end.val1 = Scenes.getTargetObjPos(options)';
      x_end.val2 = x_end.val1;
      x_end.v5 = x_goal;
      
      finalPose = FinalPoseProblem(r, g_hand, x_start(8:end), x_end.(options.model), ...
        startPoseConstraints, goalConstraints, q_nom, ...
        'capabilityMap', cm, 'graspinghand', options.graspingHand, ...
        'activecollisionoptions', ...
        struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
        'ikoptions', ikoptions, ...
        'endeffectorpoint', point_in_link_frame);
      
      optionsPlanner = struct();
      if ~isfield(optionsPlanner,'costType'), optionsPlanner.costType = 'length'; end;
      
      switch options.nTrees
        case 4
          multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
            [xStartC, xStartD], startPoseConstraints, q_nom,...
            'activecollisionoptions', ...
            struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
            'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);
          
        case 3
          multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
            [xStartC], startPoseConstraints, q_nom,...
            'activecollisionoptions', ...
            struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
            'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);
        case 2
          multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
            [], startPoseConstraints, q_nom,...
            'activecollisionoptions', ...
            struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
            'ikoptions', ikoptions, 'endeffectorpoint', point_in_link_frame);
      end

      [xGoalFull, info] = finalPose.findFinalPose(optionsPlanner);
      [multiTree, info, cost, q_path] = multiTree.rrtStar(optionsPlanner, xGoalFull);

      if info == 1
        path_length = size(q_path,2);
        xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(r.getNumVelocities(), size(q_path,2))] ));
      else
        xtraj = [];
        info = 13;
      end
      if ~isempty(xtraj), qtraj = xtraj(1:r.getNumPositions()); else, qtraj = []; end;
      if ~isempty(qtraj), s.publishTraj(qtraj, info); end;


      TA = multiTree.trees(1);
  end
  rrt_time = toc(rrt_timer);  
  
  if (info == Info.SUCCESS)
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
        %distance
      case 'rrt*'
        simVars.TA = TA;        
        simVars.info = info;
        statVars.time = rrt_time;
        statVars.numberOfVertices = TA.n;
        statVars.info = info;
        statVars.cost = cost;
        statVars.options = rmfield(options, {'robot', 'terrain'});
      case 'multiRRT'
        if size(x_end.(options.model), 2) <= 7
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
        statVars.options = rmfield(options, {'robot', 'terrain'});
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
    statVars.options = rmfield(options, {'robot', 'terrain'});
    simVars.info = info;
    fprintf('Failed to find a solution (%s)\n', info)
  end
  
end



