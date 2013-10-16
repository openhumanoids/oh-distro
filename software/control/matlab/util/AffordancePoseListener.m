classdef AffordancePoseListener
  properties
    lc
    aggregator
  end
  methods
    function obj = AffordancePoseListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel,obj.aggregator);
    end
    
    function data = getNextMessage(obj,t_ms)
      msg = obj.aggregator.getNextMessage(t_ms);
      if(isempty(msg))
        data = [];
      else
        data = AffordancePoseListener.decode(drc.affordance_collection_t(msg.data));
      end
    end
  end
  
  methods(Static)
    function data = decode(msg)
      % @param xyz     -- a 3 x naffs array, xyz(:,i) is the xyz for
      %                   affordance(i)
      % @param rpy     -- a 3 x naffs array, rpy(:,i) is the rpy for
      %                   affordance(i)
      % @param uid     -- a 1 x naffs array, uid(i) is the uid for
      %                   affordance(i) 
      naffs = msg.naffs;
      data.xyz = zeros(3,naffs);
      data.rpy = zeros(3,naffs);
      data.uid = zeros(1,naffs);
      for i = 1:naffs
        data.uid(i) = msg.affs(i).uid;
        for j = 1:3
          data.xyz(j,i) = msg.affs(i).origin_xyz(j);
          data.rpy(j,i) = msg.affs(i).origin_rpy(j);
        end
      end
    end
  end
end