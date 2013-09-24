classdef EndEffectorListener
  properties
    lc
    monitor
  end
  
  methods
    function obj = EndEffectorListener(channel_name)
      obj.lc = lcm.lcm.LCM.getSingleton();
      %obj.aggregator = lcm.lcm.MessageAggregator();
      obj.monitor = drake.util.MessageMonitor(drc.ee_goal_t,'utime');      
      obj.lc.subscribe(channel_name,obj.monitor);
    end
    
  

    function x = getNextMessage(obj,t_ms)
      %msg_raw = obj.aggregator.getNextMessage(t_ms);
      data = obj.monitor.getNextMessage(t_ms);
			if isempty(data)
				x = [];       
      else       
        msg = drc.ee_goal_t(data);
        x = EndEffectorListener.decodeEEgoal(msg);
      end      
      
%       msg_raw = obj.aggregator.getMessage();
%       if(isempty(msg_raw))
%         x = [];
%       else
%         msg = drc.ee_goal_t(msg_raw.data);
%         x = EndEffectorListener.decodeEEgoal(msg);
%       end
    end
  end
  methods(Static)    
    function x = decodeEEgoal(msg)
      pos = zeros(3,1);
      pos(1) = msg.ee_goal_pos.translation.x;
      pos(2) = msg.ee_goal_pos.translation.y;
      pos(3) = msg.ee_goal_pos.translation.z;
      quat = zeros(4,1);
      quat(1) = msg.ee_goal_pos.rotation.w;
      quat(2) = msg.ee_goal_pos.rotation.x;
      quat(3) = msg.ee_goal_pos.rotation.y;
      quat(4) = msg.ee_goal_pos.rotation.z;
      rpy = quat2rpy(quat);
      x = zeros(7,1);
      if(msg.halt_ee_controller)
        x(1) = 0;
      else
        x(1) = 1;
      end
      x(2:4) = pos;
      x(5:7) = rpy;
    end
  end
end