classdef TEMPRobotStateListener
	properties
		monitor;
    channel;
    t_cur = -1;
    x_cur = [];
	end

	methods
		function obj = TEMPRobotStateListener(channel)
      obj.channel = channel;
		  obj.monitor = drake.util.MessageMonitor(drc.robot_state_t,'utime');
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
        msg = drc.robot_state_t(data);
        x = msg.seq_id;
  	  end
    end
  end
end
