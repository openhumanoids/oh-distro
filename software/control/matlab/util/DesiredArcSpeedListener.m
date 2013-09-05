classdef DesiredArcSpeedListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = DesiredArcSpeedListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            des_speed_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(des_speed_msg)
                X = [];
            else
                [X] = DesiredArcSpeedListener.decode(drc.plan_execution_arc_speed_t(des_speed_msg.data));
            end
        end
        
    end

    methods(Static)
        function X = decode(msg)
                X.time = double(msg.utime)/1000000; % use relative time index. At the time of plan execution, this will be offset by latest sim time est from the robot state msg.        
                X.speed = msg.speed;
         end
            
    end

end
