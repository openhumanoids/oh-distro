classdef DRCPlanner
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

  methods(Static) 
    function r = constructAtlas(atlas_version)
      if nargin >= 1
        options.atlas_version = atlas_version;
      end
      options.floating = true;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      r = DRCAtlas([],options);
      r = r.removeCollisionGroupsExcept({'heel','toe','midfoot_front','midfoot_rear'},1);
      r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
      r = compile(r);
    end

    function r = constructValkyrie(valkyrie_version)
      if nargin >= 1
        options.valkyrie_version = valkyrie_version;
      end
      options.floating = true;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      options.visual = false; % loads faster
      r = OHValkyrie([], options);
      r = r.removeCollisionGroupsExcept({'heel','toe','midfoot_front','midfoot_rear'},1);
      r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
      r = compile(r);
    end

    function map_img = getDRCMapImage(heightmap, map_id, x_step, y_step, utime)
      % Convert a Drake heightmap into a DRC map image suitable for LCM broadcast
      typecheck(heightmap, 'RigidBodyHeightMapTerrain');
      map_img = drc.map_image_t();
      
      % The maps interface typically uses unsigned 8-bit integers to store the height data,
      % but Java does not appear to support unsigned values, so rather than the expected max
      % value of 255, we have to limit ourselves to 127 (actually 126 to ensure that rounding
      % does not result in an overflow). If resolution becomes a problem, we can upgrade to 
      % an int16 or float32.
      MAXINT = 126; 
      % Scale the Z values appropriately
      Z = heightmap.Z;
      Z(isinf(Z)) = 0;
      zmin = min(min(Z));
      zmax = max(max(Z));
      zrange = zmax - zmin;
      map_img.data_scale = zrange / MAXINT;
      map_img.data_shift = zmin;
      Z_scaled = (Z - map_img.data_shift) * (MAXINT / zrange);
      Z_scaled = uint8(Z_scaled);
      
      map_img.utime = utime;
      map_img.view_id = drc.data_request_t.HEIGHT_MAP_SCENE;
      map_img.map_id = map_id;
      tform = makehgtform('translate',[-heightmap.x(1)/x_step, -heightmap.y(1)/y_step, 0], 'scale', [1/x_step, 1/y_step, 1]);
      map_img.transform = tform;

      blob = drc.map_blob_t();
      blob.num_dims = 2;
      blob.dimensions = [length(heightmap.x), length(heightmap.y)];
      blob.stride_bytes = [1, blob.dimensions(1)];
      blob.compression = drc.map_blob_t.UNCOMPRESSED; % no built-in zlib support in Matlab
      blob.data_type = drc.map_blob_t.UINT8;
      blob.num_bytes = numel(Z_scaled);
      blob.data = reshape(Z_scaled, 1, []);
      map_img.blob = blob;
    end
  end

  methods
    function obj = DRCPlanner(biped, varargin)
      checkDependency('iris');
      if nargin < 1 || isempty(biped)
        biped = DRCPlanner.constructAtlas(varargin{:});
      end

      obj.biped = biped;
      
      obj.footstep_planner = StatelessFootstepPlanner();
      obj.walking_planner = StatelessWalkingPlanner();

      if isa(obj.biped, 'Atlas')
        obj.iris_planner = IRISPlanner(obj.biped,...
         Atlas(strcat(getenv('DRC_PATH'),'/models/atlas_v5/model_convex_hull.urdf'),struct('floating', true, 'atlas_version', 5)));
      elseif isa(obj.biped, 'OHValkyrie')
        obj.iris_planner = IRISPlanner(OHValkyrie([], struct('floating', true)));
      else
        warning('DRC:DRCPlanner:NoFootstepCollisionModel', 'This robot may not support upper body collision planning. Footstep plans may cause the upper body to collide with the terrain');
        obj.iris_planner = IRISPlanner(obj.biped);
      end

      obj.monitors = {};
      obj.request_channels = {};
      obj.handlers = {};
      obj.response_channels = {};
      obj.lc = lcm.lcm.LCM.getSingleton();
    end

    function run(obj)
      for j = 1:length(obj.monitors)
        obj.lc.subscribe(obj.request_channels{j}, obj.monitors{j});
      end

      req_msg = [];
      disp('Combined Planner: ready for plan requests');
      while true
        for j = 1:length(obj.monitors)
          req_msg = obj.monitors{j}.getNextMessage(5);
          if isempty(req_msg)
            continue
          end
          try
            fprintf(1, 'Handling plan on channel: %s...', obj.request_channels{j});
            plan = obj.handlers{j}(req_msg);
            if ismethod(plan, 'toLCM')
              plan = plan.toLCM();
            end
            plan.utime = now() * 24 * 60 * 60;
            obj.lc.publish(obj.response_channels{j}, plan);
          catch e
            report = e.getReport();
            disp(report)
          end
          fprintf(1, '...done\n');
        end
      end
    end

    function plan = plan_footsteps(obj, msg)
%       profile on
      msg = drc.footstep_plan_request_t(msg);
      plan = obj.footstep_planner.plan_footsteps(obj.biped, msg);
%       profile viewer
    end

    function plan = check_footsteps(obj, msg)
      msg = drc.footstep_check_request_t(msg);
      plan = obj.footstep_planner.check_footstep_plan(obj.biped, msg);
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
      coordinate_names = obj.biped.getStateFrame.getCoordinateNames;
      joint_names = coordinate_names(1:nq);
      [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true,joint_names); 
      xtraj(4:6,:) = unwrap(xtraj(4:6,:), [], 2);
      qtraj_pp = spline(ts,[zeros(nq,1), xtraj(1:nq,:), zeros(nq,1)]);

      qtraj = PPTrajectory(qtraj_pp);
      plan = QPLocomotionPlanSettings.fromQuasistaticQTraj(obj.biped, qtraj);
      
      plan = DRCQPLocomotionPlan.toLCM(plan);
    end

    function plan = configuration_traj_with_supports(obj,msg)
      msg = drc.robot_plan_with_supports_t(msg);
      nq = getNumPositions(obj.biped);
      coordinate_names = obj.biped.getStateFrame.getCoordinateNames;
      joint_names = coordinate_names(1:nq);
      [X,T,options] = RobotPlanListener.decodeRobotPlanWithSupports(msg,true,joint_names);
      X(4:6,:) = unwrap(X(4:6,:), [], 2);
      options.bodies_to_control_when_in_contact = [obj.biped.findLinkId('pelvis'), obj.biped.foot_body_id.right, obj.biped.foot_body_id.left];
      nq = obj.biped.getNumPositions();
      Q = X(1:nq,:); % extract just the q poses
      qtraj = PPTrajectory(pchip(T,Q));
      plan = QPLocomotionPlanSettings.fromQuasistaticQTraj(obj.biped, qtraj,options);
      plan = DRCQPLocomotionPlan.toLCM(plan);
    end

    function region = iris_region(obj, msg)
      msg = drc.iris_region_request_t(msg);
      region = obj.iris_planner.iris_region(msg);
    end

    function region_list = auto_iris_segmentation(obj, msg)
      msg = drc.auto_iris_segmentation_request_t(msg);
      region_list = obj.iris_planner.auto_iris_segmentation(msg);
    end

    function map_img = terrain_raycast(obj, msg)
      msg = drc.terrain_raycast_request_t(msg);
      model = RigidBodyManipulator();
      model = model.addRobotFromURDFString(char(msg.urdf.urdf_xml_string));
      heightmap = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(model,[],msg.x_min:msg.x_step:msg.x_max, msg.y_min:msg.y_step:msg.y_max, msg.scanner_height);

      map_img = CombinedPlanner.getDRCMapImage(heightmap, 0, msg.x_step, msg.y_step, msg.utime);
    end

    function obj = addRemoteSubscriptions(obj)
      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_CONTROLLER_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_controller;
      obj.response_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.robot_plan_t, 'utime');
      obj.request_channels{end+1} = 'COMMITTED_ROBOT_PLAN';
      obj.handlers{end+1} = @obj.configuration_traj;
      obj.response_channels{end+1} = 'CONFIGURATION_TRAJ';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.robot_plan_with_supports_t, 'utime');
      obj.request_channels{end+1} = 'COMMITTED_ROBOT_PLAN_WITH_SUPPORTS';
      obj.handlers{end+1} = @obj.configuration_traj_with_supports;
      obj.response_channels{end+1} = 'CONFIGURATION_TRAJ';
    end

    function obj = addBaseSubscriptions(obj)
      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_footsteps;
      obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_check_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_CHECK_REQUEST';
      obj.handlers{end+1} = @obj.check_footsteps;
      obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_TRAJ_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_traj;
      obj.response_channels{end+1} = 'WALKING_TRAJ_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_SIMULATION_DRAKE_REQUEST';
      obj.handlers{end+1} = @obj.simulate_walking_drake;
      obj.response_channels{end+1} = 'WALKING_SIMULATION_TRAJ_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.iris_region_request_t, 'utime');
      obj.request_channels{end+1} = 'IRIS_REGION_REQUEST';
      obj.handlers{end+1} = @obj.iris_region;
      obj.response_channels{end+1} = 'IRIS_REGION_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.terrain_raycast_request_t, 'utime');
      obj.request_channels{end+1} = 'TERRAIN_RAYCAST_REQUEST';
      obj.handlers{end+1} = @obj.terrain_raycast;
      obj.response_channels{end+1} = 'MAP_DEPTH';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.auto_iris_segmentation_request_t, 'utime');
      obj.request_channels{end+1} = 'AUTO_IRIS_SEGMENTATION_REQUEST';
      obj.handlers{end+1} = @obj.auto_iris_segmentation;
      obj.response_channels{end+1} = 'IRIS_SEGMENTATION_RESPONSE';
    end
  end
end
