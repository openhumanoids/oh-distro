classdef UtimeListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = UtimeListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(msg)
                X = [];
            else
                [X] = UtimeListener.decode(drc.utime_t(msg.data));
            end
        end
        
    end

    methods(Static)
        function X = decode(msg)
                X.time = double(msg.utime)/1000000; 
         end
            
    end

end
