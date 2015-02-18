classdef IRISPlanner
% Interface between the IRIS terrain grid server and the DRC planner. Handles
% computing convex regions of safe terrain based on requests received over LCM.
  properties
    terrain_map_interface
    biped
    collision_model_biped
    iris_server
  end

  methods
    function obj = IRISPlanner(biped, varargin)
      p = inputParser();
      p.addRequired('biped', @(x) isa(x, 'Biped'));
      p.addOptional('collision_model_biped', biped, @(x) isa(x, 'Biped'));
      p.addParamValue('debug', true, @isscalar);
      p.parse(biped, varargin{:});
      options = p.Results;

      obj.biped = biped;
      obj.terrain_map_interface = DRCTerrainMap(false,struct('name','IRIS Plan','status_code',6,'listen_for_foot_pose',false));

      obj.iris_server = iris.terrain_grid.Server();

      % Construct a model of the robot using the convex hull collision
      % geometry, and with all collision geometry on the left leg removed.
      % This is the model we'll use to compute an approximate volume
      % occupied by the robot when planning footsteps. 

      % Remove collision groups for the left leg (which will presumably be
      % swinging).
      % Find the left leg link names. 
      % First, compute the path from left foot to right:
      r = options.collision_model_biped;
      body_path = r.findKinematicPath(r.getFrame(r.foot_frame_id.left).body_ind,...
                                      r.getFrame(r.foot_frame_id.right).body_ind);
      % The left leg is just the first half of that path
      leg_body_ids = body_path(1:floor(length(body_path)/2))';

      if options.debug && isa(r, 'Atlas')
        valuecheck(leg_body_ids, [16, 15, 14, 13, 12, 11]);
      end
      r = r.removeCollisionGroupsExcept({},1,leg_body_ids);
      r = r.compile();
      obj.collision_model_biped = r;
    end

    function obj = setupTerrainMap(obj, q0, map_mode)
      obj.terrain_map_interface = obj.terrain_map_interface.configureFillAndOverride(obj.biped, map_mode, q0);
      [heights, px2world] = obj.terrain_map_interface.map_handle.getHeightData();

      px2world(1,end) = px2world(1,end) - sum(px2world(1,1:3)); % stupid matlab 1-indexing...
      px2world(2,end) = px2world(2,end) - sum(px2world(2,1:3));
      px2world_2x3 = px2world(1:2, [1,2,4]);

      [M, N] = meshgrid(1:size(heights, 1), 1:size(heights,2));
      sz = size(M);
      XY = px2world_2x3 * [reshape(M, 1, []); reshape(N, 1, []); ones(1, numel(M))];
      [Z, normals] = obj.terrain_map_interface.getHeight(XY);
      X = reshape(XY(1,:), sz);
      Y = reshape(XY(2,:), sz);
      Z = reshape(Z, sz);
      heightmap = iris.terrain_grid.Heightmap(X, Y, Z, normals);
      obj.iris_server.addHeightmap(0, heightmap);
    end

    function region_list = iris_region(obj, msg)
      x0 = obj.biped.getStateFrame().lcmcoder.decode(msg.initial_state);
      q0 = x0(1:obj.biped.getNumPositions());
      obj.setupTerrainMap(q0, msg.map_mode);
      collision_model = obj.collision_model_biped.getFootstepPlanningCollisionModel(q0);

      regions = iris.TerrainRegion.empty();

      yaws = zeros(size(msg.num_seed_poses));
      for j = 1:msg.num_seed_poses
        seed_pose = decodePosition3d(msg.seed_poses(j));

        xy_bounds = decodeLinCon(msg.xy_bounds(j));
        if isempty(obj.iris_server.getHeightmap(0).Z)
          if size(xy_bounds.A, 2) == 2
            xy_bounds.A(:,end+1:3) = 0;
            xy_bounds.A(end+(1:2),:) = [zeros(2), [-1;1]];
            xy_bounds.b(end+(1:2)) = [4*pi;4*pi];
          end
          regions(end+1) = iris.TerrainRegion(xy_bounds.A, xy_bounds.b, [], [], seed_pose(1:3), [0;0;1]);
          yaws(j) = q0(6) + angleDiff(q0(6), seed_pose(6));
        else
          i0 = obj.iris_server.xy2ind(0, seed_pose(1:2));
          yaws(j) = q0(6) + angleDiff(q0(6), seed_pose(6));
          
          regions(end+1) = obj.iris_server.getCSpaceRegionAtIndex(i0, yaws(j), collision_model,...
            'xy_bounds', xy_bounds, 'error_on_infeas_start', false);
        end
      end
      region_list = IRISRegionList(regions, msg.region_id, yaws);
    end

    function region_list = auto_iris_segmentation(obj, msg)
      x0 = obj.biped.getStateFrame().lcmcoder.decode(msg.initial_state);
      q0 = x0(1:obj.biped.getNumPositions());
      obj.setupTerrainMap(q0, msg.map_mode);
      if isempty(obj.iris_server.getHeightmap(0).Z)
        xy_bounds = decodeLinCon(msg.xy_bounds);
        if size(xy_bounds.A, 2) == 2
          xy_bounds.A(:,end+1:3) = 0;
          xy_bounds.A(end+(1:2),:) = [zeros(2), [-1;1]];
          xy_bounds.b(end+(1:2)) = [4*pi;4*pi];
        end
        rsole = obj.biped.feetPosition(q0).right;
        regions = iris.TerrainRegion(xy_bounds.A, xy_bounds.b, [], [], rsole(1:3), [0;0;1]);
        yaws = 0;
      else
        collision_model = obj.collision_model_biped.getFootstepPlanningCollisionModel(q0);

        options = struct();
        options.seeds = zeros(6,msg.num_seed_poses);
        for j = 1:msg.num_seed_poses
          options.seeds(:,j) = decodePosition3d(msg.seed_poses(j));
        end
        if ~isnan(msg.default_yaw)
          options.default_yaw = q0(6) + angleDiff(q0(6), msg.default_yaw);
        else
          options.default_yaw = q0(6);
        end
        if ~isnan(msg.max_slope_angle), options.max_slope_angle = msg.max_slope_angle; end
        if ~isnan(msg.max_height_variation), options.max_height_variation = msg.max_height_variation; end
        if ~isnan(msg.plane_distance_tolerance), options.plane_distance_tolerance = msg.plane_distance_tolerance; end
        if ~isnan(msg.plane_angle_tolerance), options.plane_angle_tolerance = msg.plane_angle_tolerance; end
        options.max_num_regions = msg.max_num_regions;

        regions = obj.iris_server.findSafeTerrainRegions(0, collision_model, options);
        yaws = options.default_yaw + zeros(1, length(regions));
      end
      region_list = IRISRegionList(regions, msg.region_id(1:length(regions)), yaws);
    end
  end
end


