classdef AffIndexedRobotPlanListener
    properties
        lc
        aggregator
        floating
    end
    
    methods
        function obj = AffIndexedRobotPlanListener(channel,floating)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
            if nargin > 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
        end
        
        function [X,I] = getNextMessage(obj, t_ms)
            plan_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(plan_msg)
                X = [];I=[];
            else
                [X,I] = AffIndexedRobotPlanListener.decodeAffIndexedRobotPlan(drc.aff_indexed_robot_plan_t(plan_msg.data),obj.floating);
            end
        end
        
    end
    
    methods(Static)
        function [X,I] = decodeAffIndexedRobotPlan(msg,floating)
            
            float_offset = 0;
            if floating
                float_offset = 6;
            end
            
            num_dofs = float_offset + msg.plan(1).num_joints;
            X = zeros(num_dofs*2,msg.num_states);
            I= [];
            for i=1:msg.num_states
                I(i).time = double(msg.aff_index(i).utime)/1000000; % use relative time index. At the time of plan execution, this will be offset by latest sim time est from the robot state msg.
                I(i).aff_type =msg.aff_index(i).aff_type;
                I(i).aff_uid =msg.aff_index(i).aff_uid;
                I(i).num_ees =msg.aff_index(i).num_ees;
                I(i).ee_name =msg.aff_index(i).ee_name;
                I(i).dof_name = msg.aff_index(i).dof_name;
                I(i).dof_value = msg.aff_index(i).dof_value;
                
                if floating
                    X(1,i) = msg.plan(i).origin_position.translation.x;
                    X(2,i) = msg.plan(i).origin_position.translation.y;
                    X(3,i) = msg.plan(i).origin_position.translation.z;
                    X(num_dofs+1,i) = msg.plan(i).origin_twist.linear_velocity.x;
                    X(num_dofs+2,i) = msg.plan(i).origin_twist.linear_velocity.y;
                    X(num_dofs+3,i) = msg.plan(i).origin_twist.linear_velocity.z;
                    
                    rpy = quat2rpy([msg.plan(i).origin_position.rotation.w,...
                        msg.plan(i).origin_position.rotation.x,...
                        msg.plan(i).origin_position.rotation.y,...
                        msg.plan(i).origin_position.rotation.z]);
                    
                    X(4,i) = rpy(1);
                    X(5,i) = rpy(2);
                    X(6,i) = rpy(3);
                    
                    X(num_dofs+4,i) = msg.plan(i).origin_twist.angular_velocity.x;
                    X(num_dofs+5,i) = msg.plan(i).origin_twist.angular_velocity.y;
                    X(num_dofs+6,i) = msg.plan(i).origin_twist.angular_velocity.z;
                end
                
                for j=float_offset+1:num_dofs
                    X(j,i) = msg.plan(i).joint_position(j-float_offset);
                    X(j+num_dofs,i) = msg.plan(i).joint_velocity(j-float_offset);
                end
            end
            
        end
    end
end
