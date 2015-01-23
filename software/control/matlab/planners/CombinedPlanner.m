classdef CombinedPlanner
  properties
    biped
    footstep_planner
    iris_planner
    walking_planner
    monitors
    request_channels
    handlers
    response_channels
    lc
  end

  methods (Static)
    function r = constructAtlas(atlas_version)
      if nargin >= 1
        options.atlas_version = atlas_version;
      end
      options.floating = true;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      r = DRCAtlas([],options);
      r = removeCollisionGroupsExcept(r,{'heel','toe'});
      r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
      r = compile(r);
    end

    function obj = withAtlas()
      obj = CombinedPlanner(CombinedPlanner.constructAtlas());
    end

    function r = constructValkyrie()
      options.floating = true;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      options.visual = false; % loads faster
      r = Valkyrie([], options);
      r = removeCollisionGroupsExcept(r,{'heel','toe'});
      r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
      r = compile(r);
    end

    function obj = withValkyrie()
      obj = CombinedPlanner(CombinedPlanner.constructValkyrie());
    end
  end

  methods
    function obj = CombinedPlanner(biped, varargin)
      if nargin < 1 || isempty(biped)
        biped = CombinedPlanner.constructAtlas(varargin{:});
      end

      obj.biped = biped;
      obj.footstep_planner = StatelessFootstepPlanner();
      obj.walking_planner = StatelessWalkingPlanner();
      obj.iris_planner = iris.terrain_grid.Server();
      obj.monitors = {};
      obj.request_channels = {};
      obj.handlers = {};
      obj.response_channels = {};
      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_footsteps;
      obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';

%       obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_check_request_t, 'utime');
%       obj.request_channels{end+1} = 'FOOTSTEP_CHECK_REQUEST';
%       obj.handlers{end+1} = @obj.check_footsteps;
%       obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_TRAJ_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_traj;
      obj.response_channels{end+1} = 'WALKING_TRAJ_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_CONTROLLER_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_controller;
      obj.response_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_SIMULATION_DRAKE_REQUEST';
      obj.handlers{end+1} = @obj.simulate_walking_drake;
      obj.response_channels{end+1} = 'WALKING_SIMULATION_TRAJ_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.robot_plan_t, 'utime');
      obj.request_channels{end+1} = 'COMMITTED_ROBOT_PLAN';
      obj.handlers{end+1} = @obj.configuration_traj;
      obj.response_channels{end+1} = 'CONFIGURATION_TRAJ';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.iris_region_request_t, 'utime');
      obj.request_channels{end+1} = 'IRIS_REGION_REQUEST';
      obj.handlers{end+1} = @obj.iris_region;
      obj.response_channels{end+1} = 'IRIS_REGION_RESPONSE';

    end

    function run(obj)
      for j = 1:length(obj.monitors)
        obj.lc.subscribe(obj.request_channels{j}, obj.monitors{j});
      end
      disp('Combined Planner: ready for plan requests');
      while true
        for j = 1:length(obj.monitors)
          msg = obj.monitors{j}.getNextMessage(5);
          if isempty(msg)
            continue
          end
          plan = obj.handlers{j}(msg);
          obj.lc.publish(obj.response_channels{j}, plan.toLCM());
        end
      end
    end

    function plan = plan_footsteps(obj, msg)
%       profile on
      msg = drc.footstep_plan_request_t(msg);
      plan = obj.footstep_planner.plan_footsteps(obj.biped, msg);
%       profile viewer
    end

    function plan = plan_walking_traj(obj, msg)
      msg = drc.walking_plan_request_t(msg);
      plan = obj.walking_planner.plan_walking(obj.biped, msg, true);
    end

    function plan = plan_walking_controller(obj, msg)
      msg = drc.walking_plan_request_t(msg);
      plan = obj.walking_planner.plan_walking(obj.biped, msg, false);
    end

    function plan = simulate_walking_drake(obj, msg)
      msg = drc.walking_plan_request_t(msg);
      plan = obj.walking_planner.plan_walking(obj.biped, msg, true, true);
    end
    
    function plan = configuration_traj(obj, msg)
      msg = drc.robot_plan_t(msg);
      nq = getNumPositions(obj.biped);
      joint_names = obj.biped.getStateFrame.coordinates(1:nq);
      [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names); 
      qtraj_pp = spline(ts,[zeros(nq,1), xtraj(1:nq,:), zeros(nq,1)]);
      % compute link_constraints for pelvis
      pelvis_ind = findLinkId(obj.biped,'pelvis');
      lfoot_ind = findLinkId(obj.biped,'l_foot');
      rfoot_ind = findLinkId(obj.biped,'r_foot');
      pelvis_pose = zeros(6,length(ts));
      for i=1:length(ts)
        kinsol = doKinematics(obj.biped,ppval(qtraj_pp,ts(i)));
        pelvis_pose(:,i) = forwardKin(obj.biped,kinsol,pelvis_ind,[0;0;0],1);
      end
      link_constraints(1).link_ndx = pelvis_ind;
      link_constraints(1).pt = [0;0;0];
      link_constraints(1).traj = PPTrajectory(pchip(ts,pelvis_pose));
      link_constraints(1).dtraj = fnder(link_constraints.traj);
%       link_constraints.ddtraj = fnder(link_constraints.dtraj);
      plan = ConfigurationTraj(qtraj_pp,link_constraints);
    end

    function region_list = iris_region(obj, msg)
%       profile on
%       disp('handling iris request')
      msg = drc.iris_region_request_t(msg);
      collision_model = obj.biped.getFootstepPlanningCollisionModel();
      regions = iris.TerrainRegion.empty();

      if ~obj.iris_planner.hasHeightmap(msg.map_id);
        [heights, px2world] = obj.biped.getTerrain().map_handle.getHeightData();

        px2world(1,end) = px2world(1,end) - sum(px2world(1,1:3)); % stupid matlab 1-indexing...
        px2world(2,end) = px2world(2,end) - sum(px2world(2,1:3));
        px2world_2x3 = px2world(1:2, [1,2,4]);

        [M, N] = meshgrid(1:size(heights, 1), 1:size(heights,2));
        sz = size(M);
        XY = px2world_2x3 * [reshape(M, 1, []); reshape(N, 1, []); ones(1, numel(M))];
        [Z, normals] = obj.biped.getTerrain().getHeight(XY);
        X = reshape(XY(1,:), sz);
        Y = reshape(XY(2,:), sz);
        Z = reshape(Z, sz);
        heightmap = iris.terrain_grid.Heightmap(X, Y, Z, normals);
        obj.iris_planner.addHeightmap(msg.map_id, heightmap);
      end

      for j = 1:msg.num_seed_poses
        seed_pose = decodePosition3d(msg.seed_poses(j));

        xy_bounds = decodeLinCon(msg.xy_bounds(j));
        if isempty(obj.iris_planner.getHeightmap(msg.map_id).Z)
          if size(xy_bounds.A, 2) == 2
            xy_bounds.A(:,end+1:3) = 0;
          end
          regions(end+1) = iris.TerrainRegion(xy_bounds.A, xy_bounds.b, [], [], seed_pose(1:3), [0;0;1]);
        else
          i0 = obj.iris_planner.xy2ind(msg.map_id, seed_pose(1:2));
          yaw = seed_pose(6);
          
          regions(end+1) = obj.iris_planner.getCSpaceRegionAtIndex(i0, yaw, collision_model,...
            'xy_bounds', xy_bounds, 'error_on_infeas_start', false);
        end
      end
      region_list = IRISRegionList(regions);
%       profile viewer
    end
  end
end





