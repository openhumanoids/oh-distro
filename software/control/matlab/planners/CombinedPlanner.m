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
    function obj = CombinedPlanner(biped)
      obj.biped = biped;
      obj.footstep_planner = StatelessFootstepPlanner();
      obj.monitors = {};
      obj.request_channels = {};
      obj.handlers = {};
      obj.response_channels = {};
      obj.lc = lcm.lcm.LCM.getSingleton();

      obj.monitors{end+1} = drake.util.MessageMonitor(drc.footstep_plan_request_t, 'utime');
      obj.request_channels{end+1} = 'FOOTSTEP_PLAN_REQUEST';
      obj.handlers{end+1} = @obj.footstep_planner.plan_footsteps;
      obj.response_channels{end+1} = 'FOOTSTEP_PLAN_RESPONSE';
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
          plan = obj.handlers{j}(obj.biped, msg);
          obj.lc.publish(obj.response_channels{j}, plan.toLCM());
        end
      end
    end

  end
end





