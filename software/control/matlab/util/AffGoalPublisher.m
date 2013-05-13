classdef AffGoalPublisher
    properties
        lc;
        channel;
    end
    
    methods
        function obj = AffGoalPublisher(channel)
            obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            
        end
        
        function publish(obj, X)
            if nargin < 3
                utime = now() * 24 * 60 * 60;
            end
            obj.lc.publish(obj.channel, AffGoalPublisher.encodeRobotPlan(X));
        end
        
        
    end % end methods
    
    methods(Static)
        function msg = encodeRobotPlan(X)
            if nargin < 2
                t = now() * 24 * 60 * 60;
            end
            
            msg = drc.affordance_goal_t();
            msg.utime    = t * 1000000;
            msg.aff_type = X.aff_type;
            msg.aff_uid  = X.aff_uid;
            msg.num_dofs = X.num_dofs;
            msg.dof_name=javaArray('java.lang.String', msg.num_dofs);
            msg.dof_value=zeros(1, msg.num_dofs);
            for j=1:msg.num_dofs,
              msg.dof_name(j) = java.lang.String(X.dof_name{j});
              msg.dof_value(j) = X.dof_value(j);
            end
        end
   
    end % end methods
end

