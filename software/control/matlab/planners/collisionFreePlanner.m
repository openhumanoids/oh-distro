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
  if ~isfield(options,'end_effector_name_left') || isempty(options.end_effector_name_left)
    options.end_effector_name_left = 'l_hand';
  end
  if ~isfield(options,'end_effector_name_right') || isempty(options.end_effector_name_right)
    options.end_effector_name_right = 'r_hand';
  end
  if ~isfield(options,'end_effector_pt') || isempty(options.end_effector_pt)
    options.end_effector_pt = [0; 0; 0]; 
  end
  if ~isfield(options,'left_foot_link') || isempty(options.left_foot_link)
    options.left_foot_link = 'l_foot';
  end
  if ~isfield(options,'right_foot_link') || isempty(options.right_foot_link)
    options.right_foot_link = 'r_foot';
  end
  if ~isfield(options,'goal_bias'),options.goal_bias = 0.1; end;
  if ~isfield(options,'n_smoothing_passes'),options.n_smoothing_passes = 5; end;
  if ~isfield(options,'smoothing_type'),options.smoothing_type = 'end_effector'; end;
  if ~isfield(options,'RRTMaxEdgeLength'),options.RRTMaxEdgeLength = 0.1; end;
  if ~isfield(options,'RRTBaseXYZCost'), options.RRTBaseXYZCost = 1e5; end;
  if ~isfield(options,'frozen_groups'),options.frozen_groups = {}; end;
  if ~isfield(options,'visualize'),options.visualize = true; end;
  if ~isfield(options,'display_after_every'),options.display_after_every = 10; end;
  if ~isfield(options,'verbose'),options.verbose = true; end;
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

  if options.visualize
    plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,r.getStateFrame.getCoordinateNames);
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

  active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[r.findLinkId( options.left_foot_link ), r.findLinkId( options.right_foot_link )]);
  switch options.planning_mode
    case {'rrt_connect', 'rrt'}
      % Create task-space planning tree
      TA = TaskSpaceMotionPlanningTree(r, options.end_effector_name, ...
                                          options.end_effector_pt);
      aabb_pts = r.getBody(TA.end_effector_id).getAxisAlignedBoundingBoxPoints();
      aabb_pts = bsxfun(@minus, aabb_pts, TA.end_effector_pt);
      max_radius = max(sqrt(sum(aabb_pts.^2,1)));
      TA  = TA.setOrientationWeight(2*pi*max_radius);
      TA.max_edge_length = options.RRTMaxEdgeLength;
      TA.max_length_between_constraint_checks = options.RRTMaxEdgeLength;

      % Set up task-space tolerances
      TA.angle_tol = 1*pi/180;
      TA.position_tol = 1e-3;

      % Set up collision options
      TA  = TA.setMinDistance(0.9*options.min_distance);
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

      try
        switch options.planning_mode
          case 'rrt'
            [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, options);
          case 'rrt_connect'
            [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
        end
      catch
        info = 2;
      end

      if options.verbose
        rrt_time = toc(rrt_timer);
        fprintf('  RRT:       %5.2f s\n', rrt_time);
      end

      if (info == 1) 
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
            T_smooth.interp_weight = 0.9;
            q_idx = TA.idx{TA.cspace_idx};
        end

        if (options.n_smoothing_passes > 0)
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

        % Create plan trajectory over the interval [0, 1]
        xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(q_idx,:); zeros(r.getNumVelocities(), size(q_path,2))] ));
      else
        xtraj = [];
        info = 13;
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
      l_hand_fcn = drakeFunction.kinematic.WorldPosition(r,options.end_effector_name_left);
      l_hand_fcn_all = duplicate(l_hand_fcn,(prog.N-1)*problem.n_interp_points);
      l_hand_step_lengths = smooth_norm_all(delta_r_all(l_hand_fcn_all(q_interp_all)));
      l_hand_arc_length_cost = DrakeFunctionConstraint(-Inf,Inf, ...
        k_pts*l_hand_step_lengths);
      prog = prog.addCost(l_hand_arc_length_cost,prog.q_inds);
      r_hand_fcn = drakeFunction.kinematic.WorldPosition(r,options.end_effector_name_right);
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
  if (info < 10)
    xtraj = xtraj.setOutputFrame(r.getStateFrame());
  end
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
