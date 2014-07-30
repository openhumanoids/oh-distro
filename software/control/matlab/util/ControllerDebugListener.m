classdef ControllerDebugListener
  properties
    lc
    aggregator
  end
  
  methods
    function obj = ControllerDebugListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel, obj.aggregator);
    end % end function
    
    function individual_cops = getNextMessage(obj, t_ms)
      msg = obj.aggregator.getNextMessage(t_ms);
      if isempty(msg)
        individual_cops = [];
      else
        individual_cops = ControllerDebugListener.decode(drc.controller_debug_t(msg.data));
      end
    end % end function
    
  end % end methods
  
  methods(Static)
    
    function individual_cops = decode(msg)
      individual_cops = msg.individual_cops;
    end	% end function
    
  end % end methods(static)
end
