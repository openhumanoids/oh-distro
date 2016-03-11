classdef PostureGoalListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = PostureGoalListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            msg_raw = obj.aggregator.getNextMessage(t_ms);
            
            if isempty(msg_raw)
                X = [];
            else
                msg = bot_core.joint_angles_t(msg_raw.data);
                [X] = PostureGoalListener.decode(msg);
            end
        end
        
    end
    
    methods(Static)
        function [X] = decode(msg)
            X = [];
            for j = 1:msg.num_joints,
                X(j).joint_name = msg.joint_name(j);
                X(j).joint_position = msg.joint_position(j);
            end
        end
    end
end
