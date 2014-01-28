classdef StatelessFootstepPlanner
  properties
    biped
  end

  methods
    function obj = StatelessFootstepPlanner(biped)
      typecheck(biped, 'Biped');
      obj.biped = biped;
    end

    function footsteps = plan_footsteps(obj, request)
      x0 = obj.biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      foot_orig = obj.biped.feetPosition(q0);

      terrain = obj.biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        obj.biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end

      goal_pos = StatelessFootstepPlanner.compute_goal_pos(obj.biped, request);

      params_set = {request.params};
      if request.params.leading_foot == drc.footstep_plan_params_t.LEAD_AUTO
        new_params_set = {};
        for p = params_set
          rparams = p{1};
          lparams = p{1};
          rparams.leading_foot = drc.footstep_plan_params_t.LEAD_RIGHT;
          % rparams.right_foot_lead = 1;
          lparams.leading_foot = drc.footstep_plan_params_t.LEAD_LEFT;
          % lparams.right_foot_lead = 0;
          new_params_set{end+1} = rparams;
          new_params_set{end+1} = lparams;
        end
        params_set = new_params_set;
      end
      best_steps = [];
      best_length = inf;
      best_cost = inf;
      for p = params_set
        params = struct(p{1});
        params.right_foot_lead = params.leading_foot; % for backwards compatibility
        if params.planning_mode == drc.footstep_plan_params_t.MODE_AUTO
          footsteps = footstepCollocation(obj.biped, foot_orig, goal_pos, params);
        else
          footsteps = footstepLineSearch(obj.biped, foot_orig, goal_pos.center, params);
        end
        step_vect = encodeCollocationSteps([footsteps(2:end).pos]);
        [steps, steps_rel] = decodeCollocationSteps(step_vect);
        l = length(footsteps);
        c = footstepCostFun(steps, steps_rel, goal_pos, logical(params.right_foot_lead));
        if l < best_length || c < best_cost % always prefer fewer steps
          best_steps = footsteps;
          best_length = l;
          best_cost = c;
          best_params = params;
        end
      end
      footsteps = best_steps;
      params = best_params;

      footsteps = StatelessFootstepPlanner.setStepParams(footsteps, request);
      footsteps = StatelessFootstepPlanner.mergeExistingSteps(obj.biped, footsteps, request);
      footsteps = StatelessFootstepPlanner.checkReachInfeasibility(obj.biped, footsteps, params);
    end

  end
  methods (Static=true)
    function goal_pos = compute_goal_pos(biped, request)
      if request.num_goal_steps == 0
        pos = StatelessFootstepPlanner.decodePosition3d(request.goal_pos);
        goal_pos.center = pos;
        goal_pos.right = Biped.stepCenter2FootCenter(pos, true, request.params.nom_step_width);
        goal_pos.left = Biped.stepCenter2FootCenter(pos, false, request.params.nom_step_width);
      else
        for j = 1:(min([2, request.num_goal_steps]))
          pos = StatelessFootstepPlanner.decodePosition3d(request.goal_steps(j).pos);
          if request.goal_steps(j).is_right_foot
            goal_pos.right = biped.footOrig2Contact(pos, 'center', true);
          else
            goal_pos.left = biped.footOrig2Contact(pos, 'center', false);
          end
        end
        if ~isfield(goal_pos, 'right')
          goal_pos.right = Biped.stepCenter2FootCenter(...
                                   Biped.footCenter2StepCenter(goal_pos.left, false, request.params.nom_step_width),...
                                   true, request.params.nom_step_width);
        elseif ~isfield(goal_pos, 'left')
          goal_pos.left = Biped.stepCenter2FootCenter(...
                                   Biped.footCenter2StepCenter(goal_pos.right, true, request.params.nom_step_width),...
                                   false, request.params.nom_step_width);
        end
        goal_pos.center = mean([goal_pos.right, goal_pos.left], 2);
        goal_pos.center(4:6) = goal_pos.right(4:6) + 0.5 * angleDiff(goal_pos.right(4:6), goal_pos.left(4:6));
      end
    end

    function footsteps = setStepParams(footsteps, request)
      for j = 1:length(footsteps)
        footsteps(j).id = j;
        footsteps(j).params = request.default_step_params;
        if j <= 2
          footsteps(j).pos_fixed = ones(6,1);
        else
          footsteps(j).pos_fixed = zeros(6,1);
        end
        footsteps(j).is_in_contact = true;
      end
    end

    function footsteps = mergeExistingSteps(biped, footsteps, request)
      for j = 1:request.num_existing_steps
        if request.existing_steps(j).id <= 2
          continue
        end
        ndx = request.existing_steps(j).id;
        if footsteps(ndx).is_right_foot ~= request.existing_steps(j).is_right_foot
          fprintf(1, 'Error: Right/left foot doesn''t match at ID %d', ndx);
          break
        end
        existing_pos = StatelessFootstepPlanner.decodePosition3d(request.existing_steps(j).pos);
        existing_pos = biped.footOrig2Contact(existing_pos, 'center', request.existing_steps(j).is_right_foot);
        if request.existing_steps(j).fixed_x
          footsteps(ndx).pos(1) = existing_pos(1);
        end
        if request.existing_steps(j).fixed_y
          footsteps(ndx).pos(2) = existing_pos(2);
        end
        if request.existing_steps(j).fixed_z
          footsteps(ndx).pos(3) = existing_pos(3);
        end
        if request.existing_steps(j).fixed_roll
          footsteps(ndx).pos(4) = existing_pos(4);
        end
        if request.existing_steps(j).fixed_pitch
          footsteps(ndx).pos(5) = existing_pos(5);
        end
        if request.existing_steps(j).fixed_yaw
          footsteps(ndx).pos(6) = existing_pos(6);
        end
      end
    end

    function footsteps = checkReachInfeasibility(biped, footsteps, params)
      params.right_foot_lead = logical(params.right_foot_lead);
      if ~isfield(params, 'max_line_deviation'); params.max_line_deviation = params.nom_step_width * 1.5; end
      params.forward_step = params.nom_forward_step;
      [A_reach, b_reach] = biped.getFootstepDiamondCons(true, params);
      nsteps = length(footsteps) - 1;
      [A, b, Aeq, beq, step_map] = constructCollocationAb(A_reach, b_reach, nsteps, params.right_foot_lead);
      for j = [1,2]
        footsteps(j).infeasibility = 0;
      end
      step_vect = encodeCollocationSteps([footsteps(2:end).pos]);
      violation_ineq = A * step_vect - b;
      for j = 2:nsteps
        footsteps(j+1).infeasibility = max(violation_ineq(step_map.ineq(j)));
      end
    end

    function pose = decodePosition3d(position_3d)
      pose = zeros(6,1);
      pose(1:3) = [position_3d.translation.x; position_3d.translation.y; position_3d.translation.z];
      [pose(4), pose(5), pose(6)] = quat2angle([position_3d.rotation.w,...
                                          position_3d.rotation.x,...
                                          position_3d.rotation.y,...
                                          position_3d.rotation.z], 'XYZ');
    end

    function msg = encodeFootstep(X, t)
      if nargin < 2
        t = get_timestamp_now()*1e-6;
      end

      msg = drc.footstep_t();
      msg.utime = t * 1000000;
      msg.pos = drc.position_3d_t();
      trans = drc.vector_3d_t();
      trans.x = X.pos(1);
      trans.y = X.pos(2);
      trans.z = X.pos(3);
      rot = drc.quaternion_t();
      q = rpy2quat([X.pos(4), X.pos(5), X.pos(6)]);
      rot.w = q(1);
      rot.x = q(2);
      rot.y = q(3);
      rot.z = q(4);
      msg.pos.translation = trans;
      msg.pos.rotation = rot;

      msg.id = int32(X.id);
      msg.fixed_x = X.pos_fixed(1);
      msg.fixed_y = X.pos_fixed(2);
      msg.fixed_z = X.pos_fixed(3);
      msg.fixed_roll = X.pos_fixed(4);
      msg.fixed_pitch = X.pos_fixed(5);
      msg.fixed_yaw = X.pos_fixed(6);
      msg.is_right_foot = X.is_right_foot;
      msg.is_in_contact = X.is_in_contact;
      msg.infeasibility = X.infeasibility;

      msg.params = X.params;

      if ~isfield(X, 'terrain_pts') || isempty(X.terrain_pts)
        msg.num_terrain_pts = 0;
      else
        msg.num_terrain_pts = size(X.terrain_pts, 2);
        msg.terrain_path_dist = X.terrain_pts(1,:);
        msg.terrain_height = X.terrain_pts(2,:);
      end
    end
  end
end


