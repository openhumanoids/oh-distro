classdef BasePlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = BasePlanner(BasePlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = BasePlanner(BasePlanner.constructValkyrie());
    end
  end

  methods
    function obj = BasePlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
    end
    
    function obj = addSubscriptions(obj)
      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_footsteps;
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





