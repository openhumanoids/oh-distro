classdef AffGoalListener
    properties
        lc
        aggregator
    end
    
    methods
        function obj = AffGoalListener(channel)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
        end
        
        function [X] = getNextMessage(obj, t_ms)
            goal_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(goal_msg)
                X = [];
            else
                [X] = AffGoalListener.decodeAffGoal(drc.affordance_goal_t(goal_msg.data));
            end
        end
        
    end

    methods(Static)
        function X = decodeAffGoal(msg)
                X.time = double(msg.utime)/1000000; % use relative time index. At the time of plan execution, this will be offset by latest sim time est from the robot state msg.
                X.aff_type =msg.aff_type;
                X.aff_uid =msg.aff_uid;
                X.num_dofs =msg.num_dofs;
                X.dof_name = msg.dof_name;
                X.dof_value = msg.dof_value;
         end
            
    end

end
