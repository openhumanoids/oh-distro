classdef RobotPlanPublisher
    properties
        lc;
        channel;
        floating;
        joint_names;
    end
    
    methods
        function obj = RobotPlanPublisher(channel,floating)
            obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            if nargin > 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
            
        end
        
        function obj = RobotPlanPublisher(channel,floating,joint_names)
            obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            if nargin > 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
            obj.joint_names = joint_names;
        end
        
        function publish(obj, X, utime)
            if nargin < 3
                utime = now() * 24 * 60 * 60;
            end
            obj.lc.publish(obj.channel, RobotPlanPublisher.encodeRobotPlan(X, utime));
        end
        
        function publish(obj, X,T, utime)
            if nargin < 3
                utime = now() * 24 * 60 * 60;
            end
            obj.lc.publish(obj.channel, RobotPlanPublisher.encodeRobotPlan(X,T, utime));
        end
        
    end % end methods
    
    methods(Static)
        function msg = encodeRobotPlan(X, t)
            if nargin < 2
                t = now() * 24 * 60 * 60;
            end
            
            msg = drc.robot_plan_t();
            msg.utime = t * 1000000;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            
            num_dofs = size(X,1)/2;
            
            plan = zeros(msg.num_states,1);
            float_offset = 0;
            for i=1:msg.num_states
                plan(i) = drc.robot_state_t();
                %plan(i).utime = ;%??
                if obj.floating
                    plan(i).origin_position.translation.x = X(1,i);
                    plan(i).origin_position.translation.y = X(2,i);
                    plan(i).origin_position.translation.z = X(3,i);
                    
                    q = rpy2quat([X(4,i) X(5,i) X(6,i)]);
                    % q = angle2quat(X(4,i), X(5,i), X(6,i),'XYZ');
                    
                    plan(i).origin_position.rotation.w = q(1);
                    plan(i).origin_position.rotation.x = q(2);
                    plan(i).origin_position.rotation.y = q(3);
                    plan(i).origin_position.rotation.z = q(4);
                    
                    plan(i).origin_twist.linear_velocity.x = X(num_dofs+1,i);
                    plan(i).origin_twist.linear_velocity.y = X(num_dofs+2,i);
                    plan(i).origin_twist.linear_velocity.z = X(num_dofs+3,i);
                    plan(i).origin_twist.angular_velocity.x = X(num_dofs+4,i);
                    plan(i).origin_twist.angular_velocity.y = X(num_dofs+5,i);
                    plan(i).origin_twist.angular_velocity.z = X(num_dofs+6,i);
                    
                    float_offset = 6;
                end
                
                for j=float_offset+1:num_dofs
                    if(length(obj.joint_names)>0)
                        plan(i).joint_name(j-float_offset) = obj.joint_names{j};
                    end
                    plan(i).joint_position(j-float_offset) = X(j,i);
                    plan(i).joint_velocity(j-float_offset) = X(j+num_dofs,i);
                end
            end
            
            msg.plan = plan;
        end
        
        function msg = encodeRobotPlan(X,T,t)
            if nargin < 2
                t = now() * 24 * 60 * 60;
            end
            
            msg = drc.robot_plan_t();
            msg.utime = t * 1000000;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            
            num_dofs = size(X,1)/2;
            
            plan = zeros(msg.num_states,1);
            float_offset = 0;
            for i=1:msg.num_states
                plan(i) = drc.robot_state_t();
                plan(i).utime = (T(i)*1000000+msg.utime);
                if obj.floating
                    plan(i).origin_position.translation.x = X(1,i);
                    plan(i).origin_position.translation.y = X(2,i);
                    plan(i).origin_position.translation.z = X(3,i);
                    
                    q = rpy2quat([X(4,i) X(5,i) X(6,i)]);
                    % q = angle2quat(X(4,i), X(5,i), X(6,i),'XYZ');
                    
                    plan(i).origin_position.rotation.w = q(1);
                    plan(i).origin_position.rotation.x = q(2);
                    plan(i).origin_position.rotation.y = q(3);
                    plan(i).origin_position.rotation.z = q(4);
                    
                    plan(i).origin_twist.linear_velocity.x = X(num_dofs+1,i);
                    plan(i).origin_twist.linear_velocity.y = X(num_dofs+2,i);
                    plan(i).origin_twist.linear_velocity.z = X(num_dofs+3,i);
                    plan(i).origin_twist.angular_velocity.x = X(num_dofs+4,i);
                    
                    plan(i).origin_twist.angular_velocity.y = X(num_dofs+5,i);
                    plan(i).origin_twist.angular_velocity.z = X(num_dofs+6,i);
                    
                    float_offset = 6;
                end
                
                for j=float_offset+1:num_dofs
                    if(length(obj.joint_names)>0)
                        plan(i).joint_name(j-float_offset) = obj.joint_names{j};
                    end
                    plan(i).joint_position(j-float_offset) = X(j,i);
                    plan(i).joint_velocity(j-float_offset) = X(j+num_dofs,i);
                end
            end
            
            msg.plan = plan;
        end
        
    end % end methods
end

