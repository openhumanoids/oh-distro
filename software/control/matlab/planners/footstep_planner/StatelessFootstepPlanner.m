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

      if request.params.ignore_terrain
        biped = biped.setTerrain(KinematicTerrainMap(biped, q0, true));
      else
%         biped = biped.setTerrain(biped.getTerrain().setBackupTerrain(biped, q0));
        biped = biped.setTerrain(biped.getTerrain());
      end

      terrain = biped.getTerrain();
      if ismethod(terrain, 'setMapMode')
        biped.setTerrain(terrain.setMapMode(request.params.map_command));
      end
      terrain = biped.getTerrain();
      if isprop(terrain, 'map_handle')
%         safe_regions = StatelessFootstepPlanner.computeIRISRegions(biped, request);
        load('example_iris_regions', 'safe_regions');
%         safe_regions = {struct('A', [], 'b', [])};
      else
        safe_regions = {struct('A', [], 'b', [])};
      end

      goal_pos = StatelessFootstepPlanner.compute_goal_pos(biped, request);
      if request.num_goal_steps > 2
        request.params.max_num_steps = max([1, request.params.max_num_steps - (request.num_goal_steps - 2)]);
        request.params.min_num_steps = max([1, request.params.min_num_steps - (request.num_goal_steps - 2)]);
      end

      params = struct(request.params);

      params.right_foot_lead = params.leading_foot; % for backwards compatibility
      if ~isfield(params, 'max_line_deviation');
        params.max_line_deviation = params.nom_step_width * 1.5;
      end
      corridor_pts = StatelessFootstepPlanner.corridorPoints(biped, foot_orig, goal_pos, params);
      footsteps = searchNumSteps(biped, foot_orig, goal_pos, request.goal_steps, terrain, corridor_pts, params, safe_regions);
      plan = FootstepPlan.from_collocation_results(footsteps);

      plan = StatelessFootstepPlanner.addGoalSteps(biped, plan, request);
      plan = StatelessFootstepPlanner.mergeExistingSteps(biped, plan, request);
      plan = StatelessFootstepPlanner.setStepParams(plan, request);
      plan = StatelessFootstepPlanner.snapToTerrain(biped, plan, request);
      plan = StatelessFootstepPlanner.applySwingTerrain(biped, plan, request);
      plan = StatelessFootstepPlanner.checkReachInfeasibility(biped, plan, params);
      for j = 1:length(plan.footsteps)
        plan.footsteps(j).pos = biped.footContact2Orig(plan.footsteps(j).pos, 'center', plan.footsteps(j).is_right_foot);
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
          k = nsteps - 2 + j;
          goal_step = Footstep.from_footstep_t(request.goal_steps(j));
          if j ~= 2
            assert(goal_step.is_right_foot == plan.footsteps(end-1).is_right_foot);
          else
            assert(goal_step.is_right_foot == plan.footsteps(end).is_right_foot);
          end
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
      right_foot_lead = plan.footsteps(1).is_right_foot;
      if ~isfield(params, 'max_line_deviation'); params.max_line_deviation = params.nom_step_width * 1.5; end
      params.forward_step = params.max_forward_step;
      params.nom_upward_step = 0.2; % TODO: don't hardcode this
      params.nom_downward_step = 0.15;
      [A_reach, b_reach] = biped.getFootstepDiamondCons(true, params);
      nsteps = length(plan.footsteps) - 1;
      [A, b, ~, ~, step_map] = constructCollocationAb(A_reach, b_reach, nsteps, right_foot_lead,[]);
      for j = [1,2]
        plan.footsteps(j).infeasibility = 0;
      end
      step_vect = encodeCollocationSteps([plan.footsteps(2:end).pos]);
      violation_ineq = A * step_vect - b;
      for j = 2:nsteps
        plan.footsteps(j+1).infeasibility = max(violation_ineq(step_map.ineq(j)));
      end
    end

    function corridor_pts = corridorPoints(biped, foot_orig, goal_pos, params)
      X = createOriginSteps(biped, foot_orig, params.right_foot_lead);
      st0 = X(2).pos;
      c0 = biped.footCenter2StepCenter(st0, X(2).is_right_foot, params.nom_step_width);
      goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
      dx_corridor = goal_pos.center(1:2) - c0(1:2);
      dx_corridor = dx_corridor / norm(dx_corridor);
      dy_corridor = rotmat(pi/2) * (dx_corridor);
      corridor_pts = [c0(1:2) - params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor,...
                      c0(1:2) - params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor - params.max_line_deviation * dy_corridor,...
                      goal_pos.center(1:2) + params.max_line_deviation * dx_corridor + params.max_line_deviation * dy_corridor];
    end

    function safe_regions = computeIRISRegions(biped, request, heights, px2world)
      import iris.terrain_grid.component_boundary;
      import iris.inflate_region;
      import iris.cspace.cspace3;
      import iris.cspace.project_c_space_region;
      import iris.drawing.animate_results;

      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'terrain_planning');

      if nargin < 4
        [heights, px2world] = biped.getTerrain().map_handle.getRawHeights();
      end

      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      goal_pos = StatelessFootstepPlanner.compute_goal_pos(biped, request);

      [grid, heights, px2world_2x3, world2px_2x3] = classifyTerrain(heights, px2world);
      %% Make sure the area directly under the robot is marked feasible
      corner1 = round(world2px_2x3 * [x0(1:2) - [0.35; 0.35]; 1]);
      corner2 = round(world2px_2x3 * [x0(1:2) + [0.35; 0.35]; 1]);
      box_min = min(corner1, corner2);
      box_max = max(corner1, corner2);
      grid(box_min(2):box_max(2), box_min(1):box_max(1)) = 1;
      figure(1)
      imshow(grid, 'InitialMagnification', 'fit')

      %% Find the contact points describing the robot's foot (for c-space obstacle construction)
      bot = 0.9 * bsxfun(@minus, biped.foot_bodies.right.contact_pts(1:2,:), ...
                         biped.foot_contact_offsets.right.center(1:2));
      safe_regions = {};

      %%%%%%%%%%%%%%% TODO: don't load this from file, but pull it from the request
      request.num_seed_clicks = 5;
      load('example_terrain_clicks', 'clicks');
      clicks = clicks(:,1:request.num_seed_clicks);
      %%%%%%%%%%%%%%%

      for j = 1:request.num_seed_clicks
      %%%%%%%%%%%%%%%
%         click_pos = decodePosition3d(request.seed_clicks(j));
%         cr = world2px_2x3 * [click_pos(1:2); 1];
%         r = round(cr(2)); c = round(cr(1));
%         yaw0 = click_pos(6);
        rc = clicks(:,j);
        r = round(rc(1)); c = round(rc(2));
        yaw0 = x0(6);
%         yaw0 = 0;
      %%%%%%%%%%%%%%%

        %% Restrict the search to a box containing the robot and the goal pose
        box_min = min([x0(1:2), goal_pos.center(1:2)],[], 2) - 1;
        box_max = max([x0(1:2), goal_pos.center(1:2)],[], 2) + 1;
        lb = [box_min; yaw0-pi];
        ub = [box_max; yaw0+pi];
        A_bounds = [-1,0,0;
                    0,-1,0;
                    0,0,-1;
                    1,0,0;
                    0,1,0;
                    0,0,1];
        b_bounds = [-lb;ub];

        black_edges = [];
        [edge_r, edge_c] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));
        black_edges_xy = px2world_2x3 * [edge_c'; edge_r'; ones(1,length(edge_c))];
        obs_mask = all(bsxfun(@minus, A_bounds([1,2,4,5],1:2) * black_edges_xy, b_bounds([1,2,4,5])) <= max(max(abs(bot))));
        obstacles = mat2cell(black_edges_xy(:,obs_mask) , 2, ones(1,sum(obs_mask)));
        obstacles = cspace3(obstacles, bot, linspace(yaw0-pi, yaw0+pi, 6));
        for k = 1:size(black_edges_xy,2)
          if obs_mask(k)
            lcmgl.glColor3f(0,0,0);
            lcmgl.sphere([black_edges_xy(:,k);0], 0.01, 20, 20);
          end
        end

        %% Draw the chosen start point
        lcmgl.glColor3f(0,1,0);
        lcmgl.sphere([(px2world_2x3 * [c;r;1])', heights(r,c)], 0.05, 20, 20);

        %% Actually run the convex segmentation algorithm
        iris_opts = struct('require_containment', true);
        [A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, [px2world_2x3 * [c;r;1]; yaw0], [], iris_opts);
        %   animate_results(results);
        safe_regions{end+1} = struct('A', A, 'b', b);

        %% For debugging: Draw the inner and outer polygons of the segmented space
        % The inner poly is the set of x,y points for which all yaw values are
        % contained in the polytope described by A,b. The outer poly is the set
        % of x,y points for which some yaw value is contained in the polytope.
        [inner_poly, outer_poly] = project_c_space_region(A,b);

        if ~isempty(inner_poly)
          z = ones(size(inner_poly,2)) * heights(r,c) + 0.03;
          world_xyz = [inner_poly; z];
          figure(2)
          patch(world_xyz(1,:), world_xyz(2,:), 'g', 'FaceAlpha', 0.5);
          lcmgl.glColor3f(0,1,0);
          lcmgl.glLineWidth(10);
          lcmgl.glBegin(lcmgl.LCMGL_LINES);
          for j = 1:size(world_xyz,2)-1
            lcmgl.glVertex3d(world_xyz(1,j),world_xyz(2,j),world_xyz(3,j));
            lcmgl.glVertex3d(world_xyz(1,j+1),world_xyz(2,j+1),world_xyz(3,j+1));
          end
          lcmgl.glVertex3d(world_xyz(1,end),world_xyz(2,end),world_xyz(3,end));
          lcmgl.glVertex3d(world_xyz(1,1),world_xyz(2,1),world_xyz(3,1));
          lcmgl.glEnd();
        end

        z = ones(size(outer_poly,2)) * heights(r,c) + 0.03;
        world_xyz = [outer_poly; z];
        figure(2)
        patch(world_xyz(1,:), world_xyz(2,:), 'y', 'FaceAlpha', 0.5);
        lcmgl.glColor3f(1,1,0);
        lcmgl.glLineWidth(10);
        lcmgl.glBegin(lcmgl.LCMGL_LINES);
        for j = 1:size(world_xyz,2)-1
          lcmgl.glVertex3d(world_xyz(1,j),world_xyz(2,j),world_xyz(3,j));
          lcmgl.glVertex3d(world_xyz(1,j+1),world_xyz(2,j+1),world_xyz(3,j+1));
        end
        lcmgl.glVertex3d(world_xyz(1,end),world_xyz(2,end),world_xyz(3,end));
        lcmgl.glVertex3d(world_xyz(1,1),world_xyz(2,1),world_xyz(3,1));
        lcmgl.glEnd();

      end
      lcmgl.switchBuffers();
    end

  end
end


