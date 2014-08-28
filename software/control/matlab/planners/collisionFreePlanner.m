function [xtraj,info,infeasible_constraint,xtraj_feasible,info_feasible] = collisionFreePlanner(r,t,q_seed_traj,q_nom_traj,varargin)
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

  tspan = t([1,end]);

  % uncomment to overwrite user supplied time.
  %t = linspace(tspan(1),tspan(2),4);

  if isstruct(varargin{1})
    options = varargin{1};
    varargin(1) = [];
  else
    options = struct();
  end

  if ~isfield(options,'frozen_groups'),options.frozen_groups = {}; end;
  if ~isfield(options,'visualize'),options.visualize = false; end;
  if ~isfield(options,'quiet'),options.quiet = true; end;
  if ~isfield(options,'allow_ikoptions_modification'),options.allow_ikoptions_modification = false; end;
  if ~isfield(options,'additional_t_samples'),options.additional_t_samples = []; end;
  if ~isfield(options,'acceleration_cost'),options.acceleration_cost = 1e-3; end;
  if ~isfield(options,'position_cost'),options.position_cost = 1e0; end;
  if ~isfield(options,'collision_constraint_type'), options.collision_constraint_type = 'single'; end
  if ~isfield(options,'min_distance'), options.min_distance = 0.05; end;

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
  nq = r.getNumPositions();

  for i = 1:numel(constraints)
    if ~isa(constraints{i},'PostureConstraint')
      constraints{i} = updateRobot(constraints{i},r);
    end
  end
  ikoptions = updateRobot(ikoptions,r);

  if options.visualize
    checkDependency('lcmgl');
    v = r.constructVisualizer(struct('use_contact_shapes',false));
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), ...
      'collisionFreeIKTraj');
  end

  % Adust ikoptions
  if options.allow_ikoptions_modification
    ikoptions = ikoptions.setDebug(true);
    ikoptions = ikoptions.setMajorIterationsLimit(500);
    ikoptions = ikoptions.setIterationsLimit(1e5);
    ikoptions = ikoptions.setQ(options.position_cost*ikoptions.Q);
    ikoptions = ikoptions.setQa(options.acceleration_cost*ikoptions.Qa);
    ikoptions = ikoptions.setQv(0*ikoptions.Q);
    ikoptions = ikoptions.setqdf(zeros(nq,1),zeros(nq,1));
  end

  q0 = q_seed_traj.eval(tspan(1));
  q0_constraint = PostureConstraint(r,[t(1),t(1)]);
  q0_constraint = q0_constraint.setJointLimits((1:nq)',q0,q0);
  switch options.collision_constraint_type
    case {'integrated','integrated_mex'}
      iktraj_collision_constraint = IntegratedClosestDistanceConstraint(r, t, options.min_distance, 20);
      ikoptions = ikoptions.setFixInitialState(false);
      constraints{end+1} = q0_constraint;
      ikoptions = ikoptions.setqd0(zeros(nq,1),zeros(nq,1));
      if ~strcmp(options.collision_constraint_type,'integrated_mex')
        ikoptions = ikoptions.setMex(false);
      end
    case {'interpolated','interpolated_mex'}
      iktraj_collision_constraint = InterpolatedClosestDistanceConstraint(r, t, options.min_distance, 20);
      ikoptions = ikoptions.setFixInitialState(false);
      constraints{end+1} = q0_constraint;
      ikoptions = ikoptions.setqd0(zeros(nq,1),zeros(nq,1));
      if ~strcmp(options.collision_constraint_type,'interpolated_mex')
        ikoptions = ikoptions.setMex(false);
      end
    case 'single'
      iktraj_collision_constraint = MinDistanceConstraint(r, options.min_distance);      
      ikoptions = ikoptions.setAdditionaltSamples(linspace(tspan(1),tspan(2),50));
    case 'abcdc'
      iktraj_collision_constraint = AllBodiesClosestDistanceConstraint(r, options.min_distance, 1e3);    
    otherwise
      error('test_collision_reach:badCollisionConstraintType', ...
        'Options are ''integrated'', ''interpolated'', ''single'', and ''abcdc''.');
  end

  if ~options.quiet, time = tic; end
  [xtraj_spline,info,infeasible_constraint] = inverseKinTraj(r,t,q_seed_traj,q_nom_traj,constraints{:},iktraj_collision_constraint,ikoptions);
  if ~options.quiet, fprintf('IKTraj took %f seconds\n',toc(time)); end

  % Constraints are actually checking the linear interpolation.
  switch options.collision_constraint_type
    case {'integrated','integrated_mex','interpolated','interpolated_mex'}
      xtraj = PPTrajectory(foh(xtraj_spline.getBreaks(),xtraj_spline.eval(xtraj_spline.getBreaks())));
    otherwise
      xtraj = xtraj_spline;
  end

  % Only checking for feasibility now.
  xtraj_feasible = xtraj;
  info_feasible = info;
