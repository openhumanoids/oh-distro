classdef PlanAdjustModeListener
  properties
    lc;
		monitor;
  end
    
  methods
    function obj = PlanAdjustModeListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
 		  obj.monitor = drake.util.MessageMonitor(drc.plan_adjust_mode_t(),'utime');
      obj.lc.subscribe(channel,obj.monitor);
    end
        
    function [x,t] = getNextMessage(obj, t_ms)
			data = obj.monitor.getNextMessage(t_ms);
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = drc.plan_adjust_mode_t(data);
        x = PlanAdjustModeListener.decode(msg);
			end
    end
    
    function [x,t] = getMessage(obj)
			data = obj.monitor.getMessage();
			if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = drc.plan_adjust_mode_t(data);
        x = PlanAdjustModeListener.decode(msg);
			end
		end
    
  end

  methods(Static)
    function x = decode(msg)
      x.time = double(msg.utime)/1000000; 
      x.mode =msg.mode;
    end
  end

end
