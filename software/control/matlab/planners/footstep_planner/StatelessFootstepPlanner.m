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

      if request.params.ignore_terrain
        biped = biped.setTerrain(KinematicTerrainMap(biped, q0, true));
      end

      goal_pos = StatelessFootstepPlanner.compute_goal_pos(biped, request);
      if request.num_goal_steps > 2
        request.params.max_num_steps = max([1, request.params.max_num_steps - (request.num_goal_steps - 2)]);
        request.params.min_num_steps = max([1, request.params.min_num_steps - (request.num_goal_steps - 2)]);
      end

      params_set = {struct(request.params)};
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
      end
      plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan, request);
      plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      for j = 1:length(plan.footsteps)
        plan.footsteps(j).pos = biped.footContact2Orig(plan.footsteps(j).pos, 'center', plan.footsteps(j).is_right_foot);
      end
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

    function plan = addGoalSteps(biped, plan, request)
      nsteps = length(plan.footsteps);
      if request.num_goal_steps == 0
        return;
      elseif request.num_goal_steps == 1
        goal_step = Footstep.from_footstep_t(request.goal_steps(1));
        goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', goal_step.is_right_foot);
        assert(goal_step.is_right_foot == plan.footsteps(end).is_right_foot);
        plan.footsteps(end) = goal_step;
      else
        for j = 1:request.num_goal_steps
          k = nsteps - 3 + j;
          goal_step = Footstep.from_footstep_t(request.goal_steps(j));
          assert(goal_step.is_right_foot == plan.footsteps(k).is_right_foot);
          goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', goal_step.is_right_foot);
          plan.footsteps(k) = goal_step;
        end
      end
    end

    function plan = mergeExistingSteps(biped, plan, request)
      for j = 1:request.num_existing_steps
        if request.existing_steps(j).id <= 2
          continue
        end
        ndx = request.existing_steps(j).id;
        if plan.footsteps(ndx).is_right_foot ~= request.existing_steps(j).is_right_foot
          fprintf(1, 'Error: Right/left foot doesn''t match at ID %d', ndx);
          break
        end
        existing_step = Footstep.from_footstep_t(request.existing_steps(j));
        existing_step.pos = biped.footOrig2Contact(existing_step.pos, 'center', existing_step.is_right_foot);
        if request.existing_steps(j).fixed_x
          plan.footsteps(ndx).pos(1) = existing_step.pos(1);
        end
        if request.existing_steps(j).fixed_y
          plan.footsteps(ndx).pos(2) = existing_step.pos(2);
        end
        if request.existing_steps(j).fixed_z
          plan.footsteps(ndx).pos(3) = existing_step.pos(3);
        end
        if request.existing_steps(j).fixed_roll
          plan.footsteps(ndx).pos(4) = existing_step.pos(4);
        end
        if request.existing_steps(j).fixed_pitch
          plan.footsteps(ndx).pos(5) = existing_step.pos(5);
        end
        if request.existing_steps(j).fixed_yaw
          plan.footsteps(ndx).pos(6) = existing_step.pos(6);
        end
        plan.footsteps(ndx).pos_fixed = existing_step.pos_fixed;
      end
    end

    function plan = setStepParams(plan, request)
      for j = 1:length(plan.footsteps)
        plan.footsteps(j).id = j;
        plan.footsteps(j).walking_params = request.default_step_params;
        plan.footsteps(j).is_in_contact = true;
      end
    end

    function plan = snapToTerrain(biped, plan, request)
      terrain = biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end
      nsteps = length(plan.footsteps);
      for j = 1:nsteps
        plan.footsteps(j).pos = fitStepToTerrain(biped, plan.footsteps(j).pos, 'center');
      end
    end

    function plan = applySwingTerrain(biped, plan, request)
      terrain = biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end
      nsteps = length(plan.footsteps);
      for j = 3:nsteps
        [contact_width, ~, ~] = contactVolume(biped, plan.footsteps(j-2).pos, plan.footsteps(j).pos, struct('nom_z_clearance', plan.footsteps(j).walking_params.step_height));
        plan.footsteps(j).terrain_pts = sampleSwingTerrain(biped, plan.footsteps(j-2).pos, plan.footsteps(j).pos, contact_width);
      end
    end

    function plan = checkReachInfeasibility(biped, plan, params)
      params.right_foot_lead = logical(params.right_foot_lead);
      if ~isfield(params, 'max_line_deviation'); params.max_line_deviation = params.nom_step_width * 1.5; end
      params.forward_step = params.max_forward_step;
      [A_reach, b_reach] = biped.getFootstepDiamondCons(true, params);
      nsteps = length(plan.footsteps) - 1;
      [A, b, ~, ~, step_map] = constructCollocationAb(A_reach, b_reach, nsteps, params.right_foot_lead);
      for j = [1,2]
        plan.footsteps(j).infeasibility = 0;
      end
      step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
      step_vect = reshape(step_vect, 12, []);
      right_steps = [plan.footsteps(2:end).is_right_foot];
      step_vect(12,right_steps) = -1 * step_vect(12,right_steps);
      step_vect = reshape(step_vect, [], 1);
      violation_ineq = A * step_vect - b;
      for j = 2:nsteps
        plan.footsteps(j+1).infeasibility = max(violation_ineq(step_map.ineq(j)));
      end
    end
  end
end


