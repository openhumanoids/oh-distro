classdef ManipPlanModeListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = ManipPlanModeListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            modeset_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(modeset_msg)
                X = [];
            else
                [X] = ManipPlanModeListener.decode(drc.manip_plan_control_t(modeset_msg.data));
            end
        end
        
    end

    methods(Static)
        function X = decode(msg)
                X.time = double(msg.utime)/1000000; 
                X.mode =msg.mode;
         end
            
    end

end
