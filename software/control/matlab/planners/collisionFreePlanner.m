function [xtraj,info] = collisionFreePlanner(r,t,q_seed_traj,q_nom_traj,varargin)
  % [xtraj, info] = % collisionFreePlanner(r,tspan,q_seed_traj,q_nom_traj,options,constr1,constr2,...,ikoptions)
  % 
  % @param options - [OPTIONAL] Structure that may contain the following fields
  %   * frozen_groups - Cell array of strings chosen from the following list
  %     - 'pelvis'
  %     - 'back'
  %     - 'l_arm'
  %     - 'r_arm'
  %     These indicate that the corresponding portion of the body is 'frozen'
  %     by posture constraints. Pairs of links between which collision is
  %     impossible for the given set of frozen groups will be ignored.
  %     ***********************************************************************
  %     ***********************************************************************
  %     *** NOTE: It is currently up to the user to make sure the groups    ***
  %     *** indicated by this field are actually constrained. This function ***
  %     *** will not add constraints and may yield trajectories with        ***
  %     *** collisions if groups which are listed as frozen are actually    ***
  %     *** unconstrained.  You've been warned.                             ***
  %     ***********************************************************************
  %     ***********************************************************************
  %   * visualize - Boolean indicating whether or not to draw debug visuals
  %
  %   This is where code for paralellizing the knot point loop could go.

  assert(numel(varargin)>=2);
  typecheck(varargin{end},'IKoptions');

  t0 = t(1);
  tf = t(end);
  nt = numel(t);

  q_start = q_seed_traj.eval(t(1));
  q_end = q_seed_traj.eval(t(end));

  if isstruct(varargin{1})
    options = varargin{1};
    varargin(1) = [];
  else
    options = struct();
  end

  if ~isfield(options,'planning_mode'),options.planning_mode = 'rrt_connect'; end;
  if ~isfield(options,'end_effector_name') || isempty(options.end_effector_name)
    options.end_effector_name = 'l_hand'; 
  end
  if ~isfield(options,'end_effector_pt') || isempty(options.end_effector_pt)
    options.end_effector_pt = [0; 0; 0]; 
  end
  if ~isfield(options,'goal_bias'),options.goal_bias = 0.1; end;
  if ~isfield(options,'n_smoothing_passes'),options.n_smoothing_passes = 5; end;
  if ~isfield(options,'smoothing_type'),options.smoothing_type = 'end_effector'; end;
  if ~isfield(options,'RRTOrientationWeight'),options.RRTOrientationWeight = 1; end;
  if ~isfield(options,'RRTMaxEdgeLength'),options.RRTMaxEdgeLength = 0.1; end;
  if ~isfield(options,'RRTBaseXYZCost'), options.RRTBaseXYZCost = 1e5; end;
  if ~isfield(options,'frozen_groups'),options.frozen_groups = {}; end;
  if ~isfield(options,'visualize'),options.visualize = true; end;
  if ~isfield(options,'display_after_every'),options.display_after_every = 10; end;
  if ~isfield(options,'verbose'),options.verbose = true; end;
  if ~isfield(options,'min_distance'), options.min_distance = 0.05; end;
  if ~isfield(options,'min_distance'), options.min_distance = 0.05; end;
  if ~isfield(options,'MajorIterationsLimit') 
    options.MajorIterationsLimit = 500; 
  end;
  if ~isfield(options,'N'),options.N = 100; end;
  if ~isfield(options,'MajorOptimalityTolerance') 
    options.MajorOptimalityTolerance = 1e-3; 
  end;
  if ~isfield(options,'MajorFeasibilityTolerance') 
    options.MajorFeasibilityTolerance = 5e-5; 
  end;
  if ~isfield(options,'t_max'), options.t_max = 30; end;
  if ~isfield(options,'joint_v_max'), options.joint_v_max = 30*pi/180; end;
  if ~isfield(options,'xyz_v_max'), options.xyz_v_max = 0.1; end;
  if isscalar(options.joint_v_max)
    options.joint_v_max = repmat(options.joint_v_max,r.getNumVelocities()-3,1);
  end
  if isscalar(options.xyz_v_max)
    options.xyz_v_max = repmat(options.xyz_v_max,3,1);
  end
  options.v_max = [options.xyz_v_max; options.joint_v_max];

  if options.visualize
    plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,r.getStateFrame.coordinates);
  end

  if options.verbose
    setup_timer = tic;
  end

  constraints = varargin(1:end-1);
  ikoptions = varargin{end};

  % Parse frozen groups
  if ~isempty(options.frozen_groups)
    back_frozen = any(strcmp('back',options.frozen_groups));
    pelvis_frozen = any(strcmp('back',options.frozen_groups));
    l_arm_frozen = any(strcmp('l_leg',options.frozen_groups));
    r_arm_frozen = any(strcmp('r_leg',options.frozen_groups));
    if pelvis_frozen
      r = r.addToIgnoredListOfCollisionFilterGroup({'r_leg','r_uleg','l_leg','l_uleg'},'core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'r_leg','r_uleg','l_leg','l_uleg'},'ignore_core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'r_leg','core','ignore_core'},'l_leg');
      r = r.addToIgnoredListOfCollisionFilterGroup({'l_leg','core','ignore_core'},'r_leg');
    end
    if l_arm_frozen
      r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'ignore_core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'core','ignore_core'},'l_arm');
    end
    if r_arm_frozen
      r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'ignore_core');
      r = r.addToIgnoredListOfCollisionFilterGroup({'core','ignore_core'},'r_arm');
    end
    if pelvis_frozen && back_frozen
      if l_arm_frozen
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'l_leg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'l_uleg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_leg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_uleg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_leg','l_uleg','r_leg','r_uleg'},'l_arm');
        if r_arm_frozen
          r = r.addToIgnoredListOfCollisionFilterGroup({'l_arm'},'r_arm');
          r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_arm');
        end
      end
      if r_arm_frozen
        r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_leg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'l_uleg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'r_leg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'r_arm'},'r_uleg');
        r = r.addToIgnoredListOfCollisionFilterGroup({'l_leg','l_uleg','r_leg','r_uleg'},'r_arm');
      end
    end
    r = compile(r);
  end
  nq = r.getNumPositions();

  active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[r.findLinkId('l_foot'), r.findLinkId('r_foot')]);
  switch options.planning_mode
    case {'rrt_connect', 'rrt'}
      % Create task-space planning tree
      TA = TaskSpaceMotionPlanningTree(r, options.end_effector_name, ...
                                          options.end_effector_pt);
      TA  = TA.setOrientationWeight(options.RRTOrientationWeight);
      TA.max_edge_length = options.RRTMaxEdgeLength;
      TA.max_length_between_constraint_checks = options.RRTMaxEdgeLength;

      % Set up task-space tolerances
      TA.angle_tol = 1*pi/180;
      TA.position_tol = 1e-3;

      % Set up collision options
      TA  = TA.setMinDistance(options.min_distance);
      TA.trees{TA.cspace_idx}.active_collision_options = active_collision_options;

      % Set up IK options
      TA.trees{TA.cspace_idx}.ikoptions = ikoptions;
      Q = TA.trees{TA.cspace_idx}.ikoptions.Q;
      Q(1,1) = options.RRTBaseXYZCost;
      Q(2,2) = options.RRTBaseXYZCost;
      Q(3,3) = options.RRTBaseXYZCost;
      TA.trees{TA.cspace_idx}.ikoptions = TA.trees{TA.cspace_idx}.ikoptions.setQ(Q);
      TA = TA.setNominalConfiguration(q_start);
      
      % Get initial and final end-efector poses
      kinsol = r.doKinematics(q_start);
      xyz_quat_start = r.forwardKin(kinsol,TA.end_effector_id,TA.end_effector_pt,2);
      kinsol = r.doKinematics(q_end);
      xyz_quat_goal = r.forwardKin(kinsol,TA.end_effector_id,TA.end_effector_pt,2);
      
      x_start = [xyz_quat_start;q_start];
      x_goal = [xyz_quat_goal;q_end];

      % Define xyz sampling region
      xyz_box_edge_length = 2;
      xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
      xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;
      TA = TA.setTranslationSamplingBounds(xyz_min, xyz_max);

      % Ensure that the tree's internal RigidBodyManipulators are up to date
      TA = TA.compile();
      
      % Add ik constraints to tree. Only add those that are active for the
      % entire plan
      idx_to_keep = false(size(constraints));
      for i = 1:numel(constraints)
        if ~isa(constraints{i},'PostureConstraint')
          constraints{i} = updateRobot(constraints{i},r);
        end
        idx_to_keep(i) = (constraints{i}.tspan(1) <= t0) && (tf <= constraints{i}.tspan(2));
      end
      TA = TA.addKinematicConstraint(constraints{idx_to_keep});
      
      % Create a duplicate tree for use in RRT-CONNECT
      TB = TA;
      
      if options.visualize
        TA = TA.setLCMGL('TA',[1,0,0]);
        TB = TB.setLCMGL('TB',[0,0,1]);
      end

      if options.verbose
        setup_time = toc(setup_timer);
        fprintf('Timing:\n');
        fprintf('  Setup:     %5.2f s\n', setup_time);
        rrt_timer = tic;
      end

      switch options.planning_mode
        case 'rrt'
          [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, options);
        case 'rrt_connect'
          [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
      end

      if options.verbose
        rrt_time = toc(rrt_timer);
        fprintf('  RRT:       %5.2f s\n', rrt_time);
      end

      switch options.smoothing_type
        case 'joint'
          T_smooth = TA.trees{TA.cspace_idx};
          T_smooth.max_edge_length = 15*pi/180;
          T_smooth.max_length_between_constraint_checks = 15*pi/180;
          q_idx = 1:nq;
        case 'end_effector'
          T_smooth = TA;
          % interp_weight determines how much consideration is given to joint
          % space distance during smoothing:
          %  * 0 - end-effector distance only
          %  * 1 - joint-space distance only
          T_smooth.interp_weight = 0.5;
          q_idx = TA.idx{TA.cspace_idx};
      end

      if (info == 1) && (options.n_smoothing_passes > 0)
        if options.verbose
          smoothing_timer = tic;
        end
        if options.visualize
          T_smooth = T_smooth.setLCMGL('T_smooth', TA.line_color);
        end

        [T_smooth, id_last] = T_smooth.recursiveConnectSmoothing(path_ids_A, options.n_smoothing_passes,true);
        path_ids_smooth = T_smooth.getPathToVertex(id_last);

        if options.verbose
          smoothing_time = toc(smoothing_timer);
          fprintf('  Smoothing: %5.2f s\n', smoothing_time);
          fprintf('  Total:     %5.2f s\n', setup_time+rrt_time+smoothing_time);
        end
        if options.visualize
          drawTree(TA);
          drawTree(TB);
          drawPath(T_smooth, path_ids_smooth);
        end
      end
        
      q_path = extractPath(T_smooth, path_ids_smooth);
      path_length = size(q_path,2);
        
      % Scale timing to obey joint velocity limits
      % Create initial spline
      q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(q_idx,:)));
      t = linspace(0, 1, 10*path_length);
      q_path = eval(q_traj, t);

      % Determine max joint velocity at midpoint of  each segment
      t_mid = mean([t(2:end); t(1:end-1)],1);
      v_mid = q_traj.fnder().eval(t_mid);
      scale_factor = max(abs(bsxfun(@rdivide, v_mid, options.v_max)), [], 1);

      % Adjust durations to keep velocity below max
      t_scaled = [0, cumsum(diff(t).*scale_factor)];
      tf = t_scaled(end);

      % Warp time to give gradual acceleration/deceleration
      t_scaled = tf*(-real(acos(2*t_scaled/tf-1)+pi)/2);
      [t_scaled, idx_unique] = unique(t_scaled,'stable');

      xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
    case 'old_rrt'
      var_names = [strcat({'x','y','z'},'_hand')'; strcat('w',num2cellStr(1:4))';r.getPositionFrame().coordinates];
      problem = MotionPlanningProblem(7+nq,var_names);
      
      end_effector = r.findLinkId(options.end_effector_name);
      options.floating = true;
      r_world = RigidBodyManipulator(fullfile(getDrakePath(),'systems','plants','test','FallingBrick.urdf'),options);
      r_world = r_world.removeCollisionGroupsExcept({});
      % Add world collision geometries
      for i = 1:numel(r.getBody(1).collision_geometry)
        r_world = r_world.addGeometryToBody(1,r.getBody(1).collision_geometry{i});
      end
      % Add end-effector collision geometries
      for i = 1:numel(r.getBody(end_effector).collision_geometry)
        r_world = r_world.addGeometryToBody(2, r.getBody(end_effector).collision_geometry{i});
      end
      r_world = r_world.compile();
      lcmgl = LCMGLClient('collisionFreePlanner');
      %v = r.constructVisualizer();
      %v_world = r_world.constructVisualizer(struct('use_collision_geometry',true));
      %v.draw(0,[q_start;q_start])
      %keyboard
      %v.draw(0,[q_end;q_start])
      %keyboard
      
      
      idx_to_keep = false(size(constraints));
      for i = 1:numel(constraints)
        if ~isa(constraints{i},'PostureConstraint')
          constraints{i} = updateRobot(constraints{i},r);
        end
        idx_to_keep(i) = (constraints{i}.tspan(1) <= 0) && (1 <= constraints{i}.tspan(2));
      end
      constraints = constraints(idx_to_keep);
      ikoptions = updateRobot(ikoptions,r);
      
      planning_time = tic;
      min_distance_world = options.min_distance;
      min_distance = options.min_distance;
      collision_constraint = MinDistanceConstraint(r, min_distance,active_collision_options);
      collision_constraint_world = drakeFunction.kinematic.SmoothDistancePenalty(r_world,min_distance_world);
      %collision_constraint_2 = MinDistanceConstraint(r, min_distance,active_collision_options);
      collision_constraint_2 = DrakeFunctionConstraint(0,0,drakeFunction.kinematic.SmoothDistancePenalty(r,min_distance,active_collision_options));
      %collision_constraint_2 = collision_constraint_2.generateConstraint();
      problem = problem.addConstraint(collision_constraint_2,7+(1:nq));
      
      q_nom = q_nom_traj.eval(q_nom_traj.tspan(1));
      ik_seed_pose = q_nom;
      ik_nominal_pose = q_nom;
      cost = Point(r.getPositionFrame(),10);
      for i = r.getNumBodies():-1:1
        if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
          cost(r.getBody(r.getBody(i).parent).position_num) = ...
            cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
        end
      end
      cost = cost/min(cost);
      cost = cost;
      Q = diag(cost);
      % Q = eye(nq);
      ikoptions = IKoptions(r);
      ikoptions = ikoptions.setMajorIterationsLimit(options.MajorIterationsLimit);
      ikoptions = ikoptions.setQ(Q);
      ikoptions = ikoptions.setMajorOptimalityTolerance(options.MajorOptimalityTolerance);
      
      % Get initial and final end-efector poses
      point_in_link_frame = [0; 0.3; 0];
      kinsol = r.doKinematics(q_start);
      xyz_quat_start = r.forwardKin(kinsol,end_effector,point_in_link_frame,2);
      kinsol = r.doKinematics(q_end);
      xyz_quat_goal = r.forwardKin(kinsol,end_effector,point_in_link_frame,2);
      
      position_constraint_7 = WorldPositionConstraint(r, end_effector, point_in_link_frame, xyz_quat_start(1:3), xyz_quat_start(1:3), [1.0, 1.0]);
      
      quat_constraint_8 = WorldQuatConstraint(r, end_effector, xyz_quat_start(4:7), 0, [1.0, 1.0]);
      
      active_constraints = [constraints,{position_constraint_7,quat_constraint_8}];
      
      [q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);
      
      x_start = [xyz_quat_start;q_start];
      x_goal = [xyz_quat_goal;q_end];
      xyz_box_edge_length = 3;
      xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
      xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;
      
      %orientation_weight = max(abs(xyz_max-xyz_min));%xyz_box_edge_length;
      %orientation_weight = 0*norm(xyz_max-xyz_min);%xyz_box_edge_length;
      orientation_weight = options.RRTOrientationWeight;
      posture_weight = 1e0;
      
      options.distance_metric_fcn = @(q1,q2) poseDistance(q1(1:7,:),q2(1:7,:),orientation_weight);
      options.max_length_between_constraint_checks = options.RRTMaxEdgeLength;
      options.max_edge_length = options.RRTMaxEdgeLength;
      options.interpolation_fcn = @(x1, x2, alpha) ikInterpolation(x1, x2, alpha);
      options.display_after_every = 10;
      options.display_fcn = @displayFun;
      options.verbose = true;
      
      % n_ee_poses_tried = 1;
      %sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:},collision_constraint);
      % sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:});
      % sample_prog = sample_prog.setQ(0.1*ikoptions.Q);
      % sample_prog = sample_prog.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_IterationsLimit);
      % sample_prog.setSolverOptions('snopt','MajorFeasibilityTolerance',ikoptions.SNOPT_MajorFeasibilityTolerance);
      % sample_prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
      rrt_time = tic;
      [xtraj,info,V,parent] = problem.rrt(x_start,x_goal,@sampleFunction,options);
      fprintf('rrt info: %d\n', info);
      display('RRT time:')
      toc(rrt_time);
      
      x_data = xtraj.eval(xtraj.getBreaks());
      if 1
        smoothing_time = tic;
        x_data_smoothed = x_data;
        x_data_smoothed = smoothPath(x_data_smoothed);
        x_data_smoothed = smoothPath(x_data_smoothed);
        for i = 1:options.n_smoothing_passes
          fprintf('Smoothing pass no. %d ...\n',i)
          x_data_smoothed = smoothPath2(x_data_smoothed);
          h = ones(size(x_data_smoothed,2)-1,1); h = 10*h/norm(h);
          q_data = x_data_smoothed(7+(1:nq),:);
          displayCallback(plan_publisher, size(x_data_smoothed,2), [h; q_data(:)]);
          lcmgl.glColor3f(0.8,0.8,0.8);
          for jj = 2:size(V,2)
            lcmgl.plot3(V(1, [parent(jj-1),jj]), V(2, [parent(jj-1),jj]), V(3, [parent(jj-1),jj]));
          end
          if info == 1
            lcmgl.glColor3f(0,1,0);
          else
            lcmgl.glColor3f(1,0,0);
          end
          lcmgl.plot3(x_data_smoothed(1, :), x_data_smoothed(2, :), x_data_smoothed(3, :));
          lcmgl.switchBuffers();
        end
        dist = zeros(1,size(x_data_smoothed,2)-1);
        t = zeros(1,size(x_data_smoothed,2)-1);
        for i = 1:numel(t)
          dist(i) = options.distance_metric_fcn(x_data_smoothed(:,i),x_data_smoothed(:,i+1));
        end
        velocity = 1;%-diff(0.5*cos(linspace(0,pi,numel(dist)+1)));
        t = 5*cumsum(dist./velocity)/sum(dist./velocity);
        display('Smoothing time:')
        toc(smoothing_time);
        hold on;
        %plot(x_data_smoothed(1,:),x_data_smoothed(2,:),'ro-');
        hold off;
        % q_traj = PPTrajectory(foh(linspace(0,1,numel(xtraj.getBreaks())),x_data(8:end,:)));
        q_data_smoothed = x_data_smoothed(8:end,:);
        xtraj = PPTrajectory(foh([0,t],[q_data_smoothed; zeros(r.getNumVelocities(),numel(t)+1)]));
      elseif info == 1
        options.planning_mode = 'optimization';
        q_data = x_data(7+(1:nq),:);
        t = xtraj.getBreaks();
        q_seed_traj = PPTrajectory(foh(t, q_data));
        [xtraj_opt,info] = collisionFreePlanner(r,t,q_seed_traj,q_nom_traj,options,varargin{:});
        if info < 10
          xtraj = xtraj_opt;
        end
      else
        t = xtraj.getBreaks();
        xtraj = PPTrajectory(foh(t,[x_data(8:end,:); zeros(r.getNumVelocities(),numel(t))]));
      end
    case 'optimization'
      for i = 1:numel(constraints)
        if ~isa(constraints{i},'PostureConstraint')
          constraints{i} = updateRobot(constraints{i},r);
        end
      end
      ikoptions = updateRobot(ikoptions,r);
      
      planning_time = tic;
      problem = LeggedRobotPlanningProblem(r,options);
      problem.Q = ikoptions.Q;
      problem = problem.addRigidBodyConstraint(constraints);
      prog = problem.generateQuasiStaticPlanner(nt,[0,options.t_max],q_nom_traj,q_start);
      
      k_pts = 1e2;
      q_frame = r.getPositionFrame();
      q_interp_all = drakeFunction.interpolation.Linear(q_frame,problem.n_interp_points,prog.N);
      R1 = drakeFunction.frames.realCoordinateSpace(1);
      R3 = drakeFunction.frames.realCoordinateSpace(3);
      delta_r_all = drakeFunction.Difference(R3,(prog.N-1)*problem.n_interp_points);
      smooth_norm = drakeFunction.euclidean.SmoothNorm(R3,1e-2);
      smooth_norm_all = compose(drakeFunction.Sum(R1,(prog.N-1)*problem.n_interp_points-1),duplicate(smooth_norm,(prog.N-1)*problem.n_interp_points-1));
      l_hand_fcn = drakeFunction.kinematic.WorldPosition(r,'l_hand');
      l_hand_fcn_all = duplicate(l_hand_fcn,(prog.N-1)*problem.n_interp_points);
      l_hand_step_lengths = smooth_norm_all(delta_r_all(l_hand_fcn_all(q_interp_all)));
      l_hand_arc_length_cost = DrakeFunctionConstraint(-Inf,Inf, ...
        k_pts*l_hand_step_lengths);
      prog = prog.addCost(l_hand_arc_length_cost,prog.q_inds);
      r_hand_fcn = drakeFunction.kinematic.WorldPosition(r,'r_hand');
      r_hand_fcn_all = duplicate(r_hand_fcn,(prog.N-1)*problem.n_interp_points);
      r_hand_step_lengths = smooth_norm_all(delta_r_all(r_hand_fcn_all(q_interp_all)));
      r_hand_arc_length_cost = DrakeFunctionConstraint(-Inf,Inf, ...
        k_pts*r_hand_step_lengths);
      prog = prog.addCost(r_hand_arc_length_cost,prog.q_inds);
      
      prog = prog.setSolverOptions('snopt','MajorIterationsLimit',options.MajorIterationsLimit);
      prog = prog.setSolverOptions('snopt','SuperBasicsLimit',2e3);
      %prog = prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
      %prog = prog.setSolverOptions('snopt','MajorFeasibilityTolerance',5e-5);
      prog = prog.setSolverOptions('snopt','MajorOptimalityTolerance',options.MajorOptimalityTolerance);
      prog = prog.setSolverOptions('snopt','MajorFeasibilityTolerance',options.MajorFeasibilityTolerance);
      prog = prog.setSolverOptions('snopt','IterationsLimit',5e5);
      prog = prog.setSolverOptions('snopt','LinesearchTolerance',0.1);
      if options.visualize
        prog = prog.addDisplayFunction(@(x)displayCallback(plan_publisher,nt,x),[prog.h_inds(:);prog.q_inds(:)]);
      end
      nlp_time = tic;
      [xtraj,z,F,info] = prog.solveTraj(t,q_seed_traj);
      fprintf('NLP time: %f\n',toc(nlp_time))
      fprintf('Total time: %f\n',toc(planning_time))
      t_fine = linspace(xtraj.tspan(1),xtraj.tspan(2));
      xtraj = PPTrajectory(foh(t_fine,xtraj.eval(t_fine)));
  end
  xtraj = xtraj.setOutputFrame(r.getStateFrame());

  %fprintf('Total time: %f\n',toc(planning_time))
  %t_fine = linspace(xtraj.tspan(1),xtraj.tspan(2));
  %xtraj = PPTrajectory(foh(t_fine,xtraj.eval(t_fine)));

%else
%q_traj = []
%end
  function displayFun(V,parent,last_drawn_edge_num)
    d = options.distance_metric_fcn(V,x_goal);
    [~,path] = min(d);
    while path(1)>1
      path = [parent(path(1)-1),path];
    end
    h = ones(numel(path)-1,1); h = 10*h/numel(h);
    q_data = V(7+(1:nq),path);
    displayCallback(plan_publisher, numel(path), [h; q_data(:)]);
    lcmgl.glColor3f(0.8,0.8,0.8);
    for ii = 2:size(V,2)
      lcmgl.plot3(V(1, [parent(ii-1),ii]), V(2, [parent(ii-1),ii]), V(3, [parent(ii-1),ii]));
    end
    lcmgl.glColor3f(0,0,1);
    lcmgl.plot3(V(1, path), V(2, path), V(3, path));
    lcmgl.switchBuffers();
    %v.draw(0,V(8:end,end));
    %v_world.draw(0,[V(1:3,end)-quatRotateVec(V(4:7,end),point_in_link_frame);quat2rpy(V(4:7,end))]);
    %MotionPlanningProblem.drawFirstTwoCoordinates(V,parent,last_drawn_edge_num);
  end

  function [x_interp,valid_interp] = ikInterpolation(x1,x2,alpha,tol,ang_tol,q_tol)
    if nargin < 4, tol = 0.0; end
    if nargin < 5, ang_tol = 0*pi/180; end
    if nargin < 6, q_tol = 360*pi/180; end
    x_interp = [];
    xyz = (1-alpha)*x1(1:3)+alpha*x2(1:3);
    quat = (1-alpha)*x1(4:7)+alpha*x2(4:7); %yes, I know this isn't the right way to do this
    quat = quat/norm(quat);
    q_nom_local = x1(8:end);
    q_max = max(x1(7+(1:nq)), x2(7+(1:nq))) + q_tol;
    q_min = min(x1(7+(1:nq)), x2(7+(1:nq))) - q_tol;
    
    if eval(collision_constraint_world,[xyz-quatRotateVec(quat,point_in_link_frame);quat2rpy(quat)]) < options.MajorFeasibilityTolerance
      position_constraint_7 = Point2PointDistanceConstraint(r, end_effector,1,point_in_link_frame, xyz, 0, tol, [1.0, 1.0]);
      
      quat_constraint_8 = WorldQuatConstraint(r, end_effector, quat, ang_tol, [1.0, 1.0]);

      q_bounds = PostureConstraint(r, [-inf, inf]);
      q_bounds = q_bounds.setJointLimits((1:nq)', q_min, q_max);

      
      [q, info_smooth, infeasible_constraint] = inverseKin(r, q_nom, q_nom, constraints{:}, q_bounds, position_constraint_7,quat_constraint_8,ikoptions);
      
      valid_interp = (info_smooth < 10);
      if valid_interp
        kinsol = r.doKinematics(q);
        xyz_quat = r.forwardKin(kinsol,end_effector,point_in_link_frame,2);
        x_interp = [xyz_quat; q];
        %v_world.draw(0,[xyz;quat2rpy(quat)]);
       else
         %display(info);
      end
    else
      valid_interp = false;
      %v_world.draw(0,[xyz-quatRotateVec(quat,point_in_link_frame);quat2rpy(quat)]);
      fprintf('World collision: %6.4e\n',eval(collision_constraint_world,[xyz-quatRotateVec(quat,point_in_link_frame);quat2rpy(quat)]))
    end
    %   if valid_interp% && alpha > 0.5
    % %     display('Valid interpolation');
    %     v.draw(0,q);
    % %   else
    % %     display(info);
    %   end
  end

  function x_data_smoothed = smoothPath(x_data)
    if size(x_data,2) <= 2;
      x_data_smoothed = x_data;
    else
      x0 = x_data(:,1);
      xf = x_data(:,end);
      valid_shortcut = true;
      num_interpolated_checks = ...
        ceil(options.distance_metric_fcn(x0,xf)/options.min_distance);
      x_data_smoothed = zeros(size(x_data,1),num_interpolated_checks);
      alpha = linspace(0,1,num_interpolated_checks);
      for ii = 1:num_interpolated_checks
        [x_interp,valid_shortcut] = ikInterpolation(x0,xf,alpha(ii),0,0);
        valid_shortcut = valid_shortcut && problem.checkConstraints(x_interp);
        if valid_shortcut
          x_data_smoothed(:,ii) = x_interp;
        else
          break; 
        end
      end
      if ~valid_shortcut
        mid_index = ceil(size(x_data,2)/2);
        x_data_smoothed_1 = smoothPath(x_data(:,1:mid_index));
        x_data_smoothed_2 = smoothPath(x_data(:,mid_index:end));
        x_data_smoothed = [x_data_smoothed_1(:,1:end-1),x_data_smoothed_2];
      end
    end
  end

  function x_data_smoothed = smoothPath2(x_data)
    free_indices = 2:(size(x_data,2)-1);
    step_size = 0.7;
    smooth_fraction = 1;
    x_data_smoothed = x_data;
    for ii = free_indices(rand(size(free_indices)) < smooth_fraction)
      delta_q = mean(x_data(8:end,[ii-1,ii+1]),2) - x_data(8:end,ii);
      q_new = x_data(8:end,ii) + step_size*delta_q;
      [q_new, info_smooth] = inverseKin(r, q_new, q_new, constraints{:},ikoptions);
      if info_smooth < 10 && problem.checkConstraints([NaN(7,1);q_new])
        kinsol = r.doKinematics(q_new);
        xyz_quat = r.forwardKin(kinsol,end_effector,point_in_link_frame,2);
        x_data_smoothed(:,ii) = [xyz_quat;q_new];
      end
    end
  end

  function x_data_smoothed = smoothPath3(x_data)
    if size(x_data,2) <= 2;
      x_data_smoothed = x_data;
    else
      x0 = x_data(:,1);
      xf = x_data(:,end);
      valid_shortcut = true;
      num_interpolated_checks = ...
        ceil(options.distance_metric_fcn(x0,xf)/(2*options.max_edge_length));
      x_data_smoothed = zeros(size(x_data,1),num_interpolated_checks);
      alpha = linspace(0,1,num_interpolated_checks);
      for ii = 1:num_interpolated_checks
        q_interp = alpha(ii)*x0(8:end) + (1-alpha(ii))*xf(8:end);
        valid_shortcut = valid_shortcut && problem.checkConstraints([NaN(7,1); q_interp]);
        if valid_shortcut
          kinsol = r.doKinematics(q_interp);
          xyz_quat_interp = r.forwardKin(kinsol, end_effector, point_in_link_frame, 2);
          x_data_smoothed(:,ii) = [xyz_quat_interp; q_interp];
        else
          break; 
        end
      end
      if ~valid_shortcut
        mid_index = ceil(size(x_data,2)/2);
        x_data_smoothed_1 = smoothPath3(x_data(:,1:mid_index));
        x_data_smoothed_2 = smoothPath3(x_data(:,mid_index:end));
        x_data_smoothed = [x_data_smoothed_1(:,1:end-1),x_data_smoothed_2];
      end
    end
  end

  function x_data_smoothed = smoothPath4(x_data)
    free_indices = 2:(size(x_data,2)-1);
    step_size = 0.2;
    smooth_fraction = 1;
    x_data_smoothed = x_data;
    for ii = free_indices(rand(size(free_indices)) < smooth_fraction)
      [xyz_quat_des, valid_interp] = options.interpolation_fcn(x_data(:,ii-1), x_data(:,ii+1), 0.5);
      if ~valid_interp, continue; end
      [xyz_quat_new, valid_interp] = options.interpolation_fcn(x_data(:,ii), xyz_quat_des, step_size);
      if ~valid_interp, continue; end
      delta_q = mean(x_data(8:end,[ii-1,ii+1]),2) - x_data(8:end,ii);
      q_new = x_data(8:end,ii) + step_size*delta_q;
      position_constraint_7 = Point2PointDistanceConstraint(r, end_effector,1,point_in_link_frame, xyz_quat_new(1:3), 0, 0, [1.0, 1.0]);

      quat_constraint_8 = WorldQuatConstraint(r, end_effector, xyz_quat_new(4:7), 0, [1.0, 1.0]);

      [q_new, info_smooth, infeasible_constraint] = inverseKin(r, q_new, q_new, constraints{:}, position_constraint_7,quat_constraint_8,ikoptions);
      if info_smooth < 10 && problem.checkConstraints([xyz_quat_new;q_new])
        kinsol = r.doKinematics(q_new);
        xyz_quat = r.forwardKin(kinsol,end_effector,point_in_link_frame,2);
        x_data_smoothed(:,ii) = [xyz_quat;q_new];
      end
    end
  end
                                             
  function x_sample = sampleFunction()
    xyz = (xyz_max-xyz_min).*rand(3,1)+xyz_min;
    quat = uniformlyRandomQuat();
%     have_good_quat = false;
%     while ~have_good_quat
%       quat = uniformlyRandomQuat();
%       have_good_quat = quaternionDistance(quat,xyz_quat_start(4:7)) < 0.2;
%     end
    x_sample = [xyz; quat; NaN(nq,1)];
  end
end

function dist = poseDistance(xyz_quat1,xyz_quat2,orientation_weight)
xyz1 = xyz_quat1(1:3,:);
xyz2 = xyz_quat2(1:3,:);
xyz_dist = sqrt(sum(bsxfun(@minus,xyz1,xyz2).^2,1));
quat_dist = quaternionDistance(xyz_quat1(4:7,:),xyz_quat2(4:7,:));
dist = xyz_dist + orientation_weight*quat_dist;
end

function displayCallback(publisher,N,x)
  h = x(1:N-1);
  ts = [0;cumsum(h)];
  q = reshape(x(N:end),[],N);
  x_data = [zeros(2,numel(ts));q;0*q];
  utime = now() * 24 * 60 * 60;
  snopt_info_vector = ones(1, size(x_data,2));
  publisher.publish(x_data, ts, utime, snopt_info_vector);
end
