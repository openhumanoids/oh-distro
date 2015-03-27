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

  methods (Abstract)
    addSubscriptions(obj)
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
      r = r.removeCollisionGroupsExcept({'heel','toe'},1);
      r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'listen_for_foot_pose',false)));
      r = compile(r);
    end

    function r = constructValkyrie()
      options.floating = true;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      options.visual = false; % loads faster
      r = Valkyrie([], options);
      r = r.removeCollisionGroupsExcept({'heel','toe'},1);
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
         Atlas(strcat(getenv('DRC_PATH'),'/models/atlas_v4/model_convex_hull.urdf'),struct('floating', true, 'atlas_version', 4)));
      elseif isa(obj.biped, 'Valkyrie')
        obj.iris_planner = IRISPlanner(Valkyrie([], struct('floating', true)));
      else
        warning('DRC:DRCPlanner:NoFootstepCollisionModel', 'This robot may not support upper body collision planning. Footstep plans may cause the upper body to collide with the terrain');
        obj.iris_planner = IRISPlanner(obj.biped);
      end

      obj.monitors = {};
      obj.request_channels = {};
      obj.handlers = {};
      obj.response_channels = {};
      obj.lc = lcm.lcm.LCM.getSingleton();

      obj = obj.addSubscriptions();
    end

    function run(obj)
      for j = 1:length(obj.monitors)
        obj.lc.subscribe(obj.request_channels{j}, obj.monitors{j});
      end
      status_msg = drc.system_status_t();
      status_msg.system = drc.system_status_t.PLANNING_BASE;
      status_msg.importance = drc.system_status_t.VERY_IMPORTANT;
      status_msg.frequency = drc.system_status_t.LOW_FREQUENCY;

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
            obj.lc.publish(obj.response_channels{j}, plan);
          catch e
            report = e.getReport();
            disp(report)
            status_msg.utime = get_timestamp_now();
            status_msg.value = report;
            obj.lc.publish('SYSTEM_STATUS', status_msg);
          end
          whos
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
      pelvis_pose = zeros(6,length(ts));
      for i=1:length(ts)
        kinsol = doKinematics(obj.biped,ppval(qtraj_pp,ts(i)));
        pelvis_pose(:,i) = forwardKin(obj.biped,kinsol,pelvis_ind,[0;0;0],1);
      end
      link_constraints(1).link_ndx = pelvis_ind;
      link_constraints(1).pt = [0;0;0];
      pp = pchip(ts, pelvis_pose);
      [breaks, coefs, l, k, d] = unmkpp(pp);
      link_constraints(1).ts = breaks;
      link_constraints(1).coefs = reshape(coefs, [d,l,k]);
      plan = QPLocomotionPlan.from_configuration_traj(obj.biped,qtraj_pp,link_constraints);
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
  end
end

