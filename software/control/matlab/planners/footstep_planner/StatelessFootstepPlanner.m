classdef StatelessFootstepPlanner
  methods
    function obj = StatelessFootstepPlanner()
    end
  end

  methods (Static=true)
    function plan = plan_footsteps(biped, request)
      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:biped.getNumDOF());
      foot_orig = biped.feetPosition(q0);

      biped = StatelessFootstepPlanner.configureTerrain(biped, request);
      params = struct(request.params);
      params.right_foot_lead = params.leading_foot; % for backwards compatibility

      if request.num_existing_steps > 0
        footsteps = Footstep.empty();
        for j = 1:request.num_existing_steps
          footsteps(j) = Footstep.from_footstep_t(request.existing_steps(j));
          footsteps(j).pos = biped.footOrig2Contact(footsteps(j).pos, 'center', true);
        end
        plan = FootstepPlan(footsteps, params, [], []);
      else

        goal_pos = StatelessFootstepPlanner.computeGoalPos(biped, request);
        if request.num_goal_steps > 2
          request.params.max_num_steps = max([1, request.params.max_num_steps - (request.num_goal_steps - 2)]);
          request.params.min_num_steps = max([1, request.params.min_num_steps - (request.num_goal_steps - 2)]);
        end

        safe_regions = StatelessFootstepPlanner.decodeSafeRegions(biped, request, foot_orig, goal_pos);

%         profile on
        plan = searchNumSteps(biped, foot_orig, goal_pos, request.existing_steps, request.goal_steps, params, safe_regions);
%         profile viewer
%         plan = FootstepPlan.from_collocation_results(footsteps);

        plan = StatelessFootstepPlanner.addGoalSteps(biped, plan, request);
      end
      plan = StatelessFootstepPlanner.setStepParams(plan, request);
      plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
      plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan, request);
      plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      for j = 1:length(plan.footsteps)
        plan.footsteps(j).pos = biped.footContact2Orig(plan.footsteps(j).pos, 'center', true);
      end
      plan.params = request.params;
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

    function goal_pos = computeGoalPos(biped, request)
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

    function biped = configureTerrain(biped, request)
      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:biped.getNumDOF());

      if request.params.ignore_terrain
        terrain = KinematicTerrainMap(biped, q0, true);
      else
        terrain = biped.getTerrain();
        if ismethod(terrain, 'setBackupTerrain')
          terrain = terrain.setBackupTerrain(biped, q0);
        end
      end

      if ismethod(terrain, 'setMapMode')
        terrain = terrain.setMapMode(request.params.map_command);
      end
      biped = biped.setTerrain(terrain);
    end

    function safe_regions = decodeSafeRegions(biped, request, foot_orig, goal_pos)
      if request.num_iris_regions > 0
        safe_regions = IRISRegion.empty();
        for j = 1:request.num_iris_regions
          safe_regions(end+1) = IRISRegion.from_iris_region_t(request.iris_regions(j));
        end
      else
        params = struct(request.params);
        if ~isfield(params, 'max_line_deviation');
          params.max_line_deviation = params.nom_step_width * 1.5;
        end
        corridor_pts = StatelessFootstepPlanner.corridorPoints(biped, foot_orig, goal_pos, params);
        [corr_A, corr_b] = poly2lincon(corridor_pts(1,:), corridor_pts(2,:));
        corr_A = [corr_A, zeros(size(corr_A, 1), 1)]; % convert to polytope in x y yaw
        [orig_z, orig_normal] = biped.getTerrainHeight(foot_orig.right);
        safe_regions = [IRISRegion(corr_A, corr_b, [foot_orig.right(1:2); orig_z], orig_normal)];
      end
    end

    function corridor_pts = corridorPoints(biped, foot_orig, goal_pos, params)
      goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
      c0 = mean([foot_orig.right, foot_orig.left], 2);
      dx_corridor = goal_pos.center(1:2) - c0(1:2);
      dx_corridor = dx_corridor / norm(dx_corridor);
      dy_corridor = rotmat(pi/2) * (dx_corridor);
      corridor_pts = [c0(1:2) - params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor,...
                      c0(1:2) - params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor];
    end

    function plan = addGoalSteps(biped, plan, request)
      nsteps = length(plan.footsteps);
      if request.num_goal_steps == 0
        return;
      elseif request.num_goal_steps == 1
        goal_step = Footstep.from_footstep_t(request.goal_steps(1));
        goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', true);
        assert(goal_step.body_idx == plan.footsteps(end).body_idx);
        plan.footsteps(end) = goal_step;
      else
        for j = 1:request.num_goal_steps
          k = nsteps - 2 + j;
          goal_step = Footstep.from_footstep_t(request.goal_steps(j));
          if j ~= 2
            assert(goal_step.body_idx == plan.footsteps(end-1).body_idx);
          else
            assert(goal_step.body_idx == plan.footsteps(end).body_idx);
          end
          goal_step.pos = biped.footOrig2Contact(goal_step.pos, 'center', true);
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
      if request.params.ignore_terrain
        nsteps = length(plan.footsteps) - request.num_goal_steps;
      else
        nsteps = length(plan.footsteps);
      end
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
      [A, b, ~, ~, step_map] = constructCollocationAb(biped, plan, params);
      for j = [1,2]
        plan.footsteps(j).infeasibility = 0;
      end
      step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
      violation_ineq = A * step_vect - b;
      for j = 3:length(plan.footsteps)
        plan.footsteps(j).infeasibility = max(violation_ineq(step_map.ineq(j-1)));
      end
    end
  end
end


