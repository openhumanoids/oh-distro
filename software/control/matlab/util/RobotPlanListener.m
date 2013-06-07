classdef RobotPlanListener
    properties
        lc
        aggregator
        floating
        joint_names
    end
    
    methods
        function obj = RobotPlanListener(channel,floating,joint_names)
            obj.lc = lcm.lcm.LCM.getSingleton();
            obj.aggregator = lcm.lcm.MessageAggregator();
            obj.lc.subscribe(channel, obj.aggregator);
            obj.joint_names = joint_names;
            % obj.coord_map = containers.Map(joint_names, 1:length(joint_names));
            if nargin > 2
                obj.floating = floating;
            else
                obj.floating = true;
            end
        end % end function
        
        function [X,T,G,CT] = getNextMessage(obj, t_ms)
            plan_msg = obj.aggregator.getNextMessage(t_ms);
            if isempty(plan_msg)
                X = [];T=[];G=[];
                CT.arms_control_type = 0;
                CT.legs_control_type = 0;  
            else
                [X,T,G,CT] = RobotPlanListener.decodeRobotPlan(drc.robot_plan_t(plan_msg.data),obj.floating,obj.joint_names);
            end
        end % end function
        
    end % end methods
    
    methods(Static)

        function [X,T,G,CT] = decodeRobotPlan(msg,floating,joint_names)
            float_offset = 0;
            if floating
                float_offset = 6;
            end
            coord_map = containers.Map(joint_names, 1:length(joint_names));

            num_dofs = float_offset + msg.plan(1).num_joints;
            X = zeros(num_dofs*2,msg.num_states);
            T= zeros(1,msg.num_states);
            for i=1:msg.num_states
                T(i) = double(msg.plan(i).utime)/1000000; % use relative time index. At the time of plan execution, this will be offset by latest sim time est from the robot state msg.
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
                    name = msg.plan(i).joint_name(j-float_offset);
                    coord_ndx = coord_map(char(name));
                    X(coord_ndx,i) = msg.plan(i).joint_position(j-float_offset);
                    X(coord_ndx+num_dofs,i) = msg.plan(i).joint_velocity(j-float_offset);
                    % X(j,i) = msg.plan(i).joint_position(j-float_offset);
                    % X(j+num_dofs,i) = msg.plan(i).joint_velocity(j-float_offset);
                end
            end
            
            G = [];
            for i=1:msg.num_grasp_transitions,
                G(i).time = msg.grasps(i).utime;
                G(i).affordance_uid =msg.grasps(i).affordance_uid; %(Ignore for now.)
                G(i).hand_pose = msg.grasps(i).hand_pose;
                G(i).grasp_on=msg.grasps(i).grasp_on;
                G(i).grasp_type=msg.grasps(i).grasp_type;
                G(i).power_grasp=msg.grasps(i).power_grasp;
                G(i).num_joints=msg.grasps(i).num_joints;
                G(i).joint_name=javaArray('java.lang.String', G(i).num_joints);
                for j=1:G(i).num_joints
                    G(i).joint_names(j) = msg.grasps(i).joint_name(j) ;
                    G(i).joint_position(j)=msg.grasps(i).joint_position(j);
                end
            end
            
          CT.left_arm_control_type = msg.left_arm_control_type;
          CT.right_arm_control_type = msg.right_arm_control_type;
          CT.left_leg_control_type = msg.left_leg_control_type;  
          CT.right_leg_control_type = msg.right_leg_control_type;  
            
        end	% end function
        
    end % end methods(static)
end
