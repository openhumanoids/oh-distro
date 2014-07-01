classdef LoopTestListener
	properties
		monitor;
    channel;
    t_cur = -1;
    x_cur = [];
	end

	methods
		function obj = LoopTestListener(channel)
      obj.channel = channel;
		  obj.monitor = drake.util.MessageMonitor(drc.robot_state_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
    end

		function [t] = getNextMessage(obj, t_ms)
			data = obj.monitor.getNextMessage(t_ms);
      if isempty(data)
				x = [];
        t = -1;
      else
        t = obj.monitor.getLastTimestamp();
        msg = drc.robot_state_t(data);
				x = [];
  	  end
    end
  end
end
