classdef CombinedPlanner
  properties
    biped
    footstep_planner
    monitors
    request_channels
    handlers
    response_channels
    lc
  end

  methods
    function obj = CombinedPlanner()
      options.floating = true;
      options.enable_terrainmaps = false;
      options.dt = 0.001;
      warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
      warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
      options.visual = false; % loads faster
      r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
      r = removeCollisionGroupsExcept(r,{'heel','toe'});
      if options.enable_terrainmaps
        r = setTerrain(r,DRCTerrainMap(false,struct('name','Foot Plan','status_code',6,'fill', true,'normal_radius',2,'normal_method','ransac','auto_request',true)));
      end
      r = compile(r);

      obj.biped = r;
      obj.footstep_planner = StatelessFootstepPlanner();
      obj.walking_planner = StatelessWalkingPlanner();
      obj.monitors = {};
      obj.request_channels = {};
      obj.handlers = {};
      obj.response_channels = {};
      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_footsteps;
      obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_TRAJ_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_traj;
      obj.response_channels{end+1} = 'WALKING_TRAJ_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_CONTROLLER_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_controller;
      obj.response_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';

    end

    function run(obj)
      for j = 1:length(obj.monitors)
        obj.lc.subscribe(obj.request_channels{j}, obj.monitors{j});
      end

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
      msg = drc.footstep_plan_request_t(msg);
      plan = obj.footstep_planner.plan_footsteps(obj.biped, msg);
    end

    function plan = plan_walking_traj(obj, msg)
      msg = drc.walking_plan_request_t(msg);
      plan = obj.walking_planner.plan_walking(obj, msg, true);
    end

    function plan = plan_walking_controller(obj, msg)
      msg = drc.walking_plan_request_t(msg);
      plan = obj.walking_planner.plan_walking(obj, msg, false);
    end
  end
end





