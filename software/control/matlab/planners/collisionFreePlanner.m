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

  nt = numel(t);

  q0 = q_seed_traj.eval(t(1));

  if isstruct(varargin{1})
    options = varargin{1};
    varargin(1) = [];
  else
    options = struct();
  end

  if ~isfield(options,'frozen_groups'),options.frozen_groups = {}; end;
  if ~isfield(options,'visualize'),options.visualize = false; end;
  if ~isfield(options,'quiet'),options.quiet = true; end;
  if ~isfield(options,'min_distance'), options.min_distance = 0.05; end;
  if ~isfield(options,'major_iterations_limit'), options.major_iterations_limit = 50; end;
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
  end

  r = compile(r);

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
  prog = problem.generateQuasiStaticPlanner(nt,[0,options.t_max],q_nom_traj,q0);

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

  prog = prog.setSolverOptions('snopt','MajorIterationsLimit',options.major_iterations_limit);
  prog = prog.setSolverOptions('snopt','SuperBasicsLimit',2e3);
  prog = prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
  prog = prog.setSolverOptions('snopt','MajorFeasibilityTolerance',5e-5);
  prog = prog.setSolverOptions('snopt','IterationsLimit',5e5);
  prog = prog.setSolverOptions('snopt','LinesearchTolerance',0.1);
  plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,r.getStateFrame.coordinates);
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

function displayCallback(publisher,N,x)
  h = x(1:N-1);
  ts = [0;cumsum(h)];
  q = reshape(x(N:end),[],N);
  x_data = [zeros(2,numel(ts));q;0*q];
  utime = now() * 24 * 60 * 60;
  snopt_info_vector = ones(1, size(x_data,2));
  publisher.publish(x_data, ts, utime, snopt_info_vector);
end
