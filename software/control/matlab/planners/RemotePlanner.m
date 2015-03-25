classdef RemotePlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = RemotePlanner(RemotePlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = RemotePlanner(RemotePlanner.constructValkyrie());
    end
  end

  methods
    function obj = RemotePlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
    end
    
    function obj = addSubscriptions(obj)
      obj.monitors{end+1} = drake.util.MessageMonitor(drc.walking_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'WALKING_CONTROLLER_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.plan_walking_controller;
      obj.response_channels{end+1} = 'WALKING_CONTROLLER_PLAN_RESPONSE';

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.robot_plan_t, 'utime');
      obj.request_channels{end+1} = 'COMMITTED_ROBOT_PLAN';
      obj.handlers{end+1} = @obj.configuration_traj;
      obj.response_channels{end+1} = 'CONFIGURATION_TRAJ';
    end
  end
end





