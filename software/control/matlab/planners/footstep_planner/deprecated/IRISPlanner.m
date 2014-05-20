classdef IRISPlanner
  % Static planner class to generate IRIS obstacle-free regions for
  % footstep planning
  methods
    function obj = IRISPlanner()
    end
  end

  methods (Static=true)
    function response = find_safe_regions(biped, request, heights, px2world)
      import iris.terrain_grid.component_boundary;
      import iris.inflate_region;
      import iris.cspace.cspace3;
      import iris.cspace.project_c_space_region;
      import iris.drawing.animate_results;
      MAX_BOX_SIZE_M = 4;

      x0 = biped.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:biped.getNumDOF());
      if nargin < 4
        terrain = biped.getTerrain();
        if ismethod(terrain, 'setBackupTerrain')
          terrain = terrain.setBackupTerrain(biped, q0);
        end
        if ismethod(terrain, 'setMapMode')
          terrain = terrain.setMapMode(drc.map_controller_command_t.FULL_HEIGHTMAP);
        end
        biped = biped.setTerrain(terrain);
        load('example_heights', 'heights', 'px2world');
%         [heights, px2world] = terrain.map_handle.getRawHeights();
      end
      [grid, heights, px2world_2x3, world2px_2x3] = classifyTerrain(heights, px2world);

      
      %% Find the contact points describing the robot's foot (for c-space obstacle construction)
      bot = 0.9 * bsxfun(@minus, biped.foot_bodies.right.contact_pts(1:2,:), ...
                         biped.foot_contact_offsets.right.center(1:2));
      iris_regions = IRISRegion.empty();

      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'terrain_planning');
      for i = 1:request.num_seed_poses
        click_pos = decodePosition3d(request.seed_poses(i));
        cr = world2px_2x3 * [click_pos(1:2); 1];
        r = round(cr(2)); c = round(cr(1));
        yaw0 = click_pos(6);

        %% Restrict the search to a 10m box around the seed point
        box_min = x0(1:2) - MAX_BOX_SIZE_M / 2 ;
        box_max = x0(1:2) + MAX_BOX_SIZE_M / 2;

        lb = [box_min; yaw0-pi];
        ub = [box_max; yaw0+pi];
        A_bounds = [-1,0,0;
                    0,-1,0;
                    0,0,-1;
                    1,0,0;
                    0,1,0;
                    0,0,1];
        b_bounds = [-lb;ub];

        if ~grid(r,c) % if chosen point is inside an obstacle
%           [edge_r, edge_c] = ind2sub(size(grid), find(~grid));
          edge_r = r;
          edge_c = c;
          disp('Warning: selected point is inside an obstacle');
        else
          [edge_r, edge_c] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));
        end
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
        iris_opts = struct('require_containment', false);
        start_xy = px2world_2x3 * [c;r;1];
        [A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, [start_xy; yaw0], [], iris_opts);
        %   animate_results(results);
        [start_z, start_normal] = biped.getTerrainHeight(start_xy);
        iris_regions(end+1) = IRISRegion(A, b, [start_xy; start_z], start_normal);

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

        if ~isempty(outer_poly)
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

      end
      lcmgl.switchBuffers();
      
      response = IRISRegionList(iris_regions);

    end
  end
end



