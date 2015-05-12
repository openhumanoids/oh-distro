classdef StatelessFootstepPlanner
  methods
    function obj = StatelessFootstepPlanner()
    end
  end

  methods (Static=true)
    function plan = plan_footsteps(biped, request)
      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:biped.getNumPositions());
      feet_centers = biped.feetPosition(q0);

      biped = configureDRCTerrain(biped, request.params.map_mode, q0);
      params = struct(request.params);

      if request.num_existing_steps > 0
        footsteps = Footstep.empty();
        for j = 1:request.num_existing_steps
          footsteps(j) = Footstep.from_footstep_t(request.existing_steps(j), biped);
        end
        plan = FootstepPlan(footsteps, biped, params, [], []);
      else
        goal_pos = StatelessFootstepPlanner.computeGoalPos(biped, request);
        if request.num_goal_steps > 2
          params.max_num_steps = max([1, params.max_num_steps - (request.num_goal_steps-2)]);
          params.min_num_steps = max([1, params.min_num_steps - (request.num_goal_steps-2)]);
        end

        safe_regions = StatelessFootstepPlanner.decodeSafeRegions(biped, request, feet_centers, goal_pos);

        goal_shift = round((q0(6) - goal_pos.center(6)) / (2*pi));
        if goal_shift ~= 0
          for field = {'center', 'right', 'left'}
            f = field{1};
            goal_pos.(f)(6) = goal_pos.(f)(6) + goal_shift * 2*pi;
          end
        end

        if params.planning_mode == 1
          planner = @footstepPlanner.humanoids2014;
        else
          planner = @footstepPlanner.alternatingMIQP;
        end
        plan = biped.planFootsteps(feet_centers, goal_pos, safe_regions, struct('step_params', params, 'method_handle', planner));

        plan = StatelessFootstepPlanner.addGoalSteps(biped, plan, request);
      end
      plan = StatelessFootstepPlanner.setStepParams(plan, request);
      if request.num_iris_regions > 0 && length(plan.footsteps) > 2
        plan = StatelessFootstepPlanner.snapToIRISRegions(biped, plan);
      else
        plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
      end
      plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan);
      plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      plan.params = request.params;

      % Make sure the regions attached to the plan are exactly those which were sent in the request
      plan.safe_regions = IRISRegion.empty();
      for j = 1:request.num_iris_regions
        plan.safe_regions(end+1) = IRISRegion.from_iris_region_t(request.iris_regions(j));
      end

      % Get a DRC plan with its updated LCM serialization method
      plan = DRCFootstepPlan.from_drake_footstep_plan(plan);
    end

    function plan = check_footstep_plan(biped, request)
      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:biped.getNumPositions());
      biped = configureDRCTerrain(biped, request.footstep_plan.params.map_mode, q0);
      plan = DRCFootstepPlan.from_footstep_plan_t(request.footstep_plan, biped);
      if request.snap_to_terrain
        if ~isempty(plan.safe_regions) && ~any(isnan(plan.region_order(3:end)))
          plan = StatelessFootstepPlanner.snapToIRISRegions(biped, plan);
        else
          plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
        end
        plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan);
      end
      if request.compute_infeasibility
        params = struct(plan.params);
        params.right_foot_lead = plan(1).is_right_foot;
        plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      end
    end

    function goal_pos = computeGoalPos(biped, request)
      if request.num_goal_steps == 0
        pos = decodePosition3d(request.goal_pos);
        goal_pos.center = pos;
        offs = [0; request.params.nom_step_width/2; 0];
        M = rpy2rotmat(pos(4:6));
        goal_pos.left = [pos(1:3) + M * offs; pos(4:6)];
        goal_pos.right = [pos(1:3) - M * offs; pos(4:6)];
      else
        for j = 1:(min([2, request.num_goal_steps]))
          goal_step = Footstep.from_footstep_t(request.goal_steps(j), biped);
          if request.goal_steps(j).is_right_foot
            goal_pos.right = goal_step.pos;
          else
            goal_pos.left = goal_step.pos;
          end
        end
        if ~isfield(goal_pos, 'right')
          offs = [0; -request.params.nom_step_width; 0];
          M = rpy2rotmat(goal_pos.left(4:6));
          goal_pos.right = [goal_pos.left(1:3) + M * offs; goal_pos.left(4:6)];
        elseif ~isfield(goal_pos, 'left')
          offs = [0; request.params.nom_step_width; 0];
          M = rpy2rotmat(goal_pos.right(4:6));
          goal_pos.left = [goal_pos.right(1:3) + M * offs; goal_pos.right(4:6)];
        end
        goal_pos.center = mean([goal_pos.right, goal_pos.left], 2);
        goal_pos.center(4:6) = goal_pos.right(4:6) + 0.5 * angleDiff(goal_pos.right(4:6), goal_pos.left(4:6));
      end
    end

    function safe_regions = decodeSafeRegions(biped, request, feet_centers, goal_pos)
      checkDependency('iris');
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
        corridor_pts = StatelessFootstepPlanner.corridorPoints(biped, feet_centers, goal_pos, params);
        [corr_A, corr_b] = poly2lincon(corridor_pts(1,:), corridor_pts(2,:));
        corr_A = [corr_A, zeros(size(corr_A, 1), 1)]; % convert to polytope in x y yaw
        orig_normal = rpy2rotmat(feet_centers.right(4:6)) * [0;0;1];
        safe_regions = IRISRegion(corr_A, corr_b, [], [], feet_centers.right(1:3), orig_normal);
      end
    end

    function corridor_pts = corridorPoints(biped, feet_centers, goal_pos, params)
      goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
      c0 = mean([feet_centers.right, feet_centers.left], 2);
      dx_corridor = goal_pos.center(1:2) - c0(1:2);
      dx_corridor = dx_corridor / norm(dx_corridor);
      dy_corridor = rotmat(pi/2) * (dx_corridor);
      corridor_pts = [c0(1:2) - params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor,...
                      c0(1:2) - params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor];
    end

    function plan = addGoalSteps(biped, plan, request)
      n_unmodified_steps = max(2, length(plan.footsteps)-2);
      if request.num_goal_steps == 0
        return;
      elseif request.num_goal_steps == 1
        goal_step = Footstep.from_footstep_t(request.goal_steps(1), biped);
        if (goal_step.frame_id ~= plan.footsteps(end).frame_id)
          plan.footsteps(end+1) = plan.footsteps(end-1);
          plan.footsteps(end).id = plan.footsteps(end-1).id + 1;
        end
        assert(goal_step.frame_id == plan.footsteps(end).frame_id);
        plan.footsteps(end) = goal_step;
        plan.region_order(end) = nan;
      else
        for j = 1:request.num_goal_steps
          goal_step = Footstep.from_footstep_t(request.goal_steps(j), biped);
          if j == 1 && (goal_step.frame_id ~= plan.footsteps(end-1).frame_id)
            plan.footsteps(end+1) = plan.footsteps(end-1);
            plan.footsteps(end).id = plan.footsteps(end-1).id + 1;
            n_unmodified_steps = max(2, length(plan.footsteps)-2);
          end
          k = n_unmodified_steps + j;
          plan.footsteps(k) = goal_step;
          plan.region_order(k) = nan;
        end
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
      nsteps = length(plan.footsteps);
      for j = 3:nsteps
        if ~plan.footsteps(j).pos_fixed(3)
          plan.footsteps(j) = fitStepToTerrain(biped, plan.footsteps(j));
        end
      end
    end

    function plan = snapToIRISRegions(biped, plan)
      % Note: we skip steps 1 and 2 (which are the current feet poses and don't need to be snapped)
      for j = 3:length(plan.footsteps)
        region = plan.safe_regions(plan.region_order(j));

        if ~any(plan.footsteps(j).pos_fixed(4:5))
          R_step = rpy2rotmat(plan.footsteps(j).pos(4:6));
          step_normal = R_step * [0;0;1];

          ax = reshape(cross(step_normal, region.normal), 3, 1);
          if norm(ax) > 1e-3
            theta = asin(norm(ax) / (norm(region.normal) * norm(step_normal)));
            R_snap = axis2rotmat([ax; theta]);
            R_step = R_snap * R_step;
            plan.footsteps(j).pos(4:6) = rotmat2rpy(R_step);
          end
        end

        if ~plan.footsteps(j).pos_fixed(3)
          % n' * xyz = n' * p
          % n' * p = n1*x + n2*y + n3*z
          % z = (n'*p - n1*x - n2*y) / n3;
          plan.footsteps(j).pos(3) = (region.normal' * region.point - region.normal(1:2)' * plan.footsteps(j).pos(1:2)) / region.normal(3);
        end
      end
    end

    function plan = applySwingTerrain(biped, plan)
      nsteps = length(plan.footsteps);
      for j = 3:nsteps

        [~, contact_width] = contactVolume(biped, ...
                                              plan.footsteps(j-2), ...
                                              plan.footsteps(j));
        plan.footsteps(j).terrain_pts = sampleSwingTerrain(biped, plan.footsteps(j-2), plan.footsteps(j), contact_width/2, struct());
      end
    end

    function plan = checkReachInfeasibility(biped, plan, params)
      params = biped.applyDefaultFootstepParams(params);
      [A, b, ~, ~, step_map] = constructCollocationAb(biped, plan, params);
      for j = [1,2]
        plan.footsteps(j).infeasibility = 0;
      end
      steps = plan.step_matrix();
      if length(plan.footsteps) > 2
        step_vect = StatelessFootstepPlanner.encodeCollocationSteps(steps(:,2:end));
        violation_ineq = A * step_vect - b;
        for j = 3:length(plan.footsteps)
          plan.footsteps(j).infeasibility = max(violation_ineq(step_map.ineq(j-1)));
        end
      end
    end

    function x = encodeCollocationSteps(steps)
      nsteps = size(steps, 2);
      x = zeros(12,nsteps);
      x(1:6,:) = steps;
      x(7:12,1) = steps(:,1);
      for j = 2:nsteps
        R = rotmat(-steps(6,j-1));
        x(7:12,j) = [R * (steps(1:2,j) - steps(1:2,j-1));
                    steps(3:6,j) - steps(3:6,j-1)];
      end
      x = reshape(x, [], 1);
    end

  end
end


