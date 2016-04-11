classdef CandidateRobotPosePublisher
    properties
        lc;
        channel;
        floating;
        joint_names;
    end
    
    methods

        function obj = CandidateRobotPosePublisher(channel,floating,joint_names)
            obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            if nargin > 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
            obj.joint_names = joint_names;
        end

        function publish(obj,varargin)
            switch nargin
                case 2
                    utime = get_timestamp_now();% equivalent to bot_timestamp_now();
                case 3
                    utime= varargin{2};
                otherwise
                    error('Incorrect usage of publish in CandidateRobotPosePublisher. Undefined number of varargin.')
            end
            X = varargin{1};
            msg = encodeRobotPose(obj,X,utime);
            obj.lc.publish(obj.channel,msg);
        end


        function msg = encodeRobotPose(obj,X,t)
            if nargin < 3
                t = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            num_dofs = (size(X,1))/2;
            
            msg = bot_core.robot_state_t();
            msg.utime = t;
            msg.pose = bot_core.position_3d_t();
            msg.pose.translation = bot_core.vector_3d_t();
            msg.pose.rotation = bot_core.quaternion_t();
            msg.pose.translation.x = X(1);
            msg.pose.translation.y = X(2);
            msg.pose.translation.z = X(3);
            
            q = rpy2quat([X(4) X(5) X(6)]);
            msg.pose.rotation.w = q(1);
            msg.pose.rotation.x = q(2);
            msg.pose.rotation.y = q(3);
            msg.pose.rotation.z = q(4);
            
            msg.twist = bot_core.twist_t();
            msg.twist.linear_velocity = bot_core.vector_3d_t();
            msg.twist.angular_velocity = bot_core.vector_3d_t();
            msg.twist.linear_velocity.x = X(num_dofs+1);
            msg.twist.linear_velocity.y = X(num_dofs+2);
            msg.twist.linear_velocity.z = X(num_dofs+3);
            msg.twist.angular_velocity.x = X(num_dofs+4);
            msg.twist.angular_velocity.y = X(num_dofs+5);
            msg.twist.angular_velocity.z = X(num_dofs+6);
            

            float_offset = 6;
            msg.num_joints = num_dofs - float_offset;
            msg.joint_name=javaArray('java.lang.String', msg.num_joints);
            msg.joint_position=zeros(1,msg.num_joints);
            msg.joint_velocity=zeros(1,msg.num_joints);
            msg.joint_effort=zeros(1,msg.num_joints);
            for j=float_offset+1:num_dofs,
                msg.joint_name(j-float_offset) = java.lang.String(obj.joint_names{j});
                msg.joint_position(j-float_offset) = X(j);
                msg.joint_velocity(j-float_offset) = X(j+num_dofs);
            end
                   
            msg.force_torque = bot_core.force_torque_t();
        end
        
    end % end methods
end
