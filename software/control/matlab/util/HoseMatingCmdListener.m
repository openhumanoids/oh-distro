classdef HoseMatingCmdListener
  properties
    monitor;
    channel;
  end
  
  methods
    function obj = HoseMatingCmdListener(channel)
      obj.channel = channel;
      obj.monitor = drake.util.MessageMonitor(drc.hose_mating_cmd_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
    end
    
    function cmd = getNextMessage(obj,t_ms)
      data = obj.monitor.getNextMessage(t_ms);
      if(isempty(data))
        cmd = [];
      else
        msg = drc.hose_mating_cmd_t(data);
        cmd = msg.cmd;
      end
    end
  end
end