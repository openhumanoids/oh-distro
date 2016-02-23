classdef AtlasBehaviorModeListener 
	properties
		monitor;
    channel;
    t_cur = -1;
    x_cur = [];
	end

	methods
		function obj = AtlasBehaviorModeListener(channel)
      if nargin < 1
        channel = 'ATLAS_STATUS';
      end
      obj.channel = channel;
		  obj.monitor = drake.util.MessageMonitor(atlas.status_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
    end

		function [x,t] = getNextMessage(obj, t_ms)
			data = obj.monitor.getNextMessage(t_ms);
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = atlas.status_t(data);
        x = AtlasBehaviorModeListener.decode(msg);
			end
    end
    
    function [x,t] = getMessage(obj)
			data = obj.monitor.getMessage();
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = atlas.status_t(data);
        x = AtlasBehaviorModeListener.decode(msg);
			end
		end
    
  end
  
  methods(Static)
    function data = decode(msg)
      data = msg.behavior;
    end
  end
  
  properties(Constant)
  	BEHAVIOR_NONE       = 0;
  	BEHAVIOR_FREEZE     = 1;
  	BEHAVIOR_STAND_PREP = 2;
  	BEHAVIOR_STAND      = 3;
  	BEHAVIOR_WALK       = 4;
  	BEHAVIOR_STEP       = 5;
  	BEHAVIOR_MANIPULATE = 6;
  	BEHAVIOR_USER       = 7;    
  end
end
