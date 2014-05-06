function [xtraj, info, infeasible_constraint] = collisionFreeIKTraj(r,t,q_seed_traj,q_nom_traj,varargin)
  % [xtraj, info] = % collisionFreeIKTraj(r,t,q_seed_traj,q_nom_traj,options,constr1,constr2,...,ikoptions)
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

  assert(numel(varargin)>=2);
  typecheck(varargin{end},'IKoptions');

  if isstruct(varargin{1})
    options = varargin{1};
    varargin(1) = [];
  else
    options = struct();
  end

  if ~isfield(options,'frozen_groups'),options.frozen_groups = {}; end;
  if ~isfield(options,'visualize'),options.visualize = false; end;
  if ~isfield(options,'quiet'),options.quiet = true; end;

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
  nq = r.getNumDOF();

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
  ikoptions = ikoptions.setMajorIterationsLimit(500);
  ikoptions = ikoptions.setQa(1e-9*eye(nq));
  ikoptions = ikoptions.setQ(0*ikoptions.Q);
  ikoptions = ikoptions.setQv(0*ikoptions.Q);

  %q_nom_traj = ConstantTrajectory(q_nom_traj.eval(t(1)));

  num_collision_check_samples = 500;
  t_fine = linspace(t(1), t(end), num_collision_check_samples);

  additional_t_samples = [];
  n_additional_t_samples = numel(additional_t_samples);
  info = NaN;
  planning_done = false;

  while ~planning_done % Additional time samples loop
    % Set additional t samples
    ikoptions = ikoptions.setAdditionaltSamples(additional_t_samples);

    if ~options.quiet
      disp('Running inverseKinTraj...');
      fprintf(['nt                     = %d\n', ...
               'n_additional_t_samples = %d\n\n'],numel(t), n_additional_t_samples);
    end

    last_info = info;
    [xtraj, info, infeasible_constraint] = inverseKinTraj(r, t, q_seed_traj, q_nom_traj, constraints{:}, ikoptions);
    disp(info);
    if (info > 10)
      % inverseKinTraj failed. Terminate and return info
      %q_seed_traj = q_nom_traj;
      if (1 || info == last_info)
        if ~options.quiet
          display(infeasibleConstraintMsg(infeasible_constraint));
        end
        break;
      end
      return;
    else
      % inverseKinTraj succeeds. Proceed to collision checking
      %q_seed_traj = xtraj;
      planning_done = true;
    end;

    q_fine = xtraj.eval(t_fine);
    q_fine = q_fine(1:nq,:);

    % Check for collisions on a fine discretization of the trajectory
    distance = zeros(1,size(q_fine,2));
    for i = 1:size(q_fine,2)
      [ptsA,ptsB] = r.allCollisions(q_fine(:,i));
      planning_done = planning_done && isempty(ptsA);
      if ~isempty(ptsA)
        distance(i) = max(sum((ptsA-ptsB).^2,1));
      end
    end

    if ~planning_done 
      % Then we either have collisions or inverseKinTraj failed.
      [max_penetration_distance,max_penetration_idx] = max(distance);
      if max_penetration_distance > 0
        if max_penetration_distance < 0.01;
          q_seed_traj = xtraj;
        end
        % The trajectory returned by inverseKinTraj contained collisions!
        % Add additional t_samples at the following points:
        %   * time of first penetration
        %   * time of maximum penetration
        %   * time of last penetration
        %   * halfway between t0 and time of first penetration
        %   * halfway between time of last penetration and tf
        if options.visualize
          [ptsA,ptsB] = r.allCollisions(q_fine(:,max_penetration_idx));
          for j = 1:size(ptsA,2), 
            lcmgl.glColor3f(1,0,0);
            lcmgl.sphere(ptsA(:,j),0.02,20,20);
            lcmgl.glColor3f(0,0,1);
            lcmgl.sphere(ptsB(:,j),0.02,20,20); 
          end;
          lcmgl.switchBuffers();
          v.draw(0,[q_fine(:,max_penetration_idx);zeros(nq,1)]);
          xtraj = xtraj.setOutputFrame(v.getInputFrame());
          playback(v, xtraj);
        end
        first_penetration_idx = find(distance>0, 1, 'first');
        last_penetration_idx = find(distance>0, 1, 'last');
        additional_t_samples(end+1) = t_fine(floor(first_penetration_idx/2));
        additional_t_samples(end+1) = t_fine(first_penetration_idx);
        additional_t_samples(end+1) = t_fine(max_penetration_idx);
        additional_t_samples(end+1) = t_fine(last_penetration_idx);
        additional_t_samples(end+1) = t_fine(floor((num_collision_check_samples + last_penetration_idx)/2));
        %n_additional_t_samples = numel(additional_t_samples);
        %[~,unique_idx] = unique(round(additional_t_samples/0.01));
        %additional_t_samples = additional_t_samples(unique_idx);
        n_additional_t_samples = numel(additional_t_samples);
        if options.visualize
          figure(7);
          plot(t_fine,distance,'-b',additional_t_samples,zeros(size(additional_t_samples)),'r.');
        end
      end
    end
  end
end
