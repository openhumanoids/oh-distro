classdef PresetPostureGoalListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = PresetPostureGoalListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            goal_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(goal_msg)
                X = [];
            else
                [X] = PresetPostureGoalListener.decode(drc.robot_posture_preset_t(goal_msg.data));
            end
        end
        
    end

    methods(Static)
        function X = decode(msg)
                X.time = double(msg.utime)/1000000; 
                X.preset =msg.preset;
         end
            
    end

end
