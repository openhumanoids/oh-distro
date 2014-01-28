classdef StatelessFootstepPlanner
  methods
    function obj = StatelessFootstepPlanner()
    end
  end

  methods (Static=true)
    function plan = plan_footsteps(biped, request)
      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      foot_orig = biped.feetPosition(q0);

      goal_pos = StatelessFootstepPlanner.compute_goal_pos(biped, request);
      if request.num_goal_steps > 2
        request.params.max_num_steps = max([1, request.params.max_num_steps - (request.num_goal_steps - 2)]);
        request.params.min_num_steps = max([1, request.params.min_num_steps - (request.num_goal_steps - 2)]);
      end

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
        if request.num_goal_steps == 0
          params.allow_odd_num_steps = true;
          params.allow_even_num_steps = true;
        else
          if xor(request.goal_steps(end).is_right_foot, params.right_foot_lead)
            params.allow_even_num_steps = true;
            params.allow_odd_num_steps = false;
          else
            params.allow_even_num_steps = false;
            params.allow_odd_num_steps = true;
          end
        end
        footsteps = footstepCollocation(biped, foot_orig, goal_pos, params);
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
      plan = FootstepPlan.from_collocation_results(best_steps);
      params = best_params;

      plan = StatelessFootstepPlanner.addGoalSteps(biped, plan, request);
      plan = StatelessFootstepPlanner.mergeExistingSteps(biped, plan, request);
      plan = StatelessFootstepPlanner.setStepParams(plan, request);
      if ~request.params.ignore_terrain
        plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
        plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan, request);
      end
      plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
    end

    function plan = check_footstep_plan(biped, request)
      plan = FootstepPlan.from_footstep_plan_t(request.footstep_plan);
      if request.snap_to_terrain
        plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
        plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan, request);
      end
      if request.compute_infeasibility
        params = struct(request.footstep_params);
        params.right_foot_lead = plan(1).is_right_foot;
        plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      end
    end

    function goal_pos = compute_goal_pos(biped, request)
      if request.num_goal_steps == 0
        pos = decodePosition3d(request.goal_pos);
        goal_pos.center = pos;
        goal_pos.right = Biped.stepCenter2FootCenter(pos, true, request.params.nom_step_width);
        goal_pos.left = Biped.stepCenter2FootCenter(pos, false, request.params.nom_step_width);
      else
        for j = 1:(min([2, request.num_goal_steps]))
          pos = decodePosition3d(request.goal_steps(j).pos);
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
        footsteps(j).walking_params = request.default_step_params;
        footsteps(j).is_in_contact = true;
      end
    end

    function footsteps = addGoalSteps(biped, footsteps, request)
      nsteps = length(footsteps);
      if request.num_goal_steps == 0
        return;
      elseif request.num_goal_steps == 1
        goal_step = Footstep.from_footstep_t(request.goal_steps(1));
        goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', goal_step.is_right_foot);
        if footsteps(end).is_right_foot == goal_step.is_right_foot
          k = nsteps;
        else
          k = nsteps - 1;
        end
        footsteps(k) = goal_step;
      else
        if footsteps(end-1).is_right_foot ~= request.goal_steps(1).is_right_foot
          request.goal_steps = StatelessFootstepPlanner.pairReverse(request.goal_steps);
        end
        for j = 1:request.num_goal_steps
          k = nsteps - 2 + j;
          goal_step = Footstep.from_footstep_t(request.goal_steps(j));
          goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', goal_step.is_right_foot);
          footsteps(k) = goal_step;
        end
      end
    end

    function y = pairReverse(x)
      % If x is [a, b, c, d], returns [b, a, d, c]
      y = zeros(size(x));
      y(1:2:end) = x(2:2:end);
      y(2:2:end) = x(1:2:end);
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
        existing_step = Footstep.from_footstep_t(request.existing_steps(j));
        existing_step.pos = biped.footOrig2Contact(existing_step.pos, 'center', existing_step.is_right_foot);
        if request.existing_steps(j).fixed_x
          footsteps(ndx).pos(1) = existing_step.pos(1);
        end
        if request.existing_steps(j).fixed_y
          footsteps(ndx).pos(2) = existing_step.pos(2);
        end
        if request.existing_steps(j).fixed_z
          footsteps(ndx).pos(3) = existing_step.pos(3);
        end
        if request.existing_steps(j).fixed_roll
          footsteps(ndx).pos(4) = existing_step.pos(4);
        end
        if request.existing_steps(j).fixed_pitch
          footsteps(ndx).pos(5) = existing_step.pos(5);
        end
        if request.existing_steps(j).fixed_yaw
          footsteps(ndx).pos(6) = existing_step.pos(6);
        end
        footsteps(ndx).pos_fixed = existing_step.pos_fixed;
      end
    end

    function plan = snapToTerrain(biped, plan, request)
      terrain = biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end
      nsteps = length(plan);
      for j = 1:nsteps
        plan(j).pos = fitStepToTerrain(biped, plan(j).pos, 'center');
      end
    end

    function plan = applySwingTerrain(biped, plan, request)
      terrain = biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end
      nsteps = length(plan);
      for j = 3:nsteps
        [contact_width, ~, ~] = contactVolume(biped, plan(j-2).pos, plan(j).pos, struct('nom_z_clearance', plan(j).walking_params.step_height));
        plan(j).terrain_pts = sampleSwingTerrain(biped, plan(j-2).pos, plan(j).pos, contact_width);
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
  end
end


