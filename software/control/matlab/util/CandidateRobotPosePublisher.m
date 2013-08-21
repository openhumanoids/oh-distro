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
            if nargin < 3
                utime = now() * 24 * 60 * 60;
            end                  
            switch nargin
                case 3
                    X = varargin{1};
                    utime= varargin{2};
                    msg = encodeRobotPose(obj,X,utime);
                    obj.lc.publish(obj.channel,msg);
                otherwise
                    error('Incorrect usage of publish in CandidateRobotPosePublisher. Undefined number of varargin.')
            end
        end


        function msg = encodeRobotPose(obj,X,t)
            if nargin < 4
                t = now() * 24 * 60 * 60;
            end
            offset = 0;
            num_dofs = (size(X,1)-offset)/2;
            
            msg = drc.robot_state_t();
            msg.utime = t * 1000000;
            msg.robot_name = 'atlas';
            i=1;
            msg.pose = drc.position_3d_t();
            msg.pose.translation = drc.vector_3d_t();
            msg.pose.rotation = drc.quaternion_t();
            msg.pose.translation.x = X(1,i);
            msg.pose.translation.y = X(2,i);
            msg.pose.translation.z = X(3,i);
            
            q = rpy2quat([X(4,i) X(5,i) X(6,i)]);                  
            msg.pose.rotation.w = q(1);
            msg.pose.rotation.x = q(2);
            msg.pose.rotation.y = q(3);
            msg.pose.rotation.z = q(4);
            
            msg.twist = drc.twist_t();
            msg.twist.linear_velocity = drc.vector_3d_t();
            msg.twist.angular_velocity = drc.vector_3d_t();   
            msg.twist.linear_velocity.x = X(offset+num_dofs+1,i);
            msg.twist.linear_velocity.y = X(offset+num_dofs+2,i);
            msg.twist.linear_velocity.z = X(offset+num_dofs+3,i);
            msg.twist.angular_velocity.x = X(offset+num_dofs+4,i);
            msg.twist.angular_velocity.y = X(offset+num_dofs+5,i);
            msg.twist.angular_velocity.z = X(offset+num_dofs+6,i);     
            

            float_offset = 6;
            msg.num_joints = num_dofs - float_offset;
            msg.joint_name=javaArray('java.lang.String', msg.num_joints);
            msg.joint_position=zeros(1,msg.num_joints);
            msg.joint_velocity=zeros(1,msg.num_joints);
            msg.joint_effort=zeros(1,msg.num_joints);
            for j=float_offset+1:num_dofs,
                msg.joint_name(j-float_offset) = java.lang.String(obj.joint_names{j});
                msg.joint_position(j-float_offset) = X(j+offset,i);
                msg.joint_velocity(j-float_offset) = X(j+offset+num_dofs,i);
            end
                   
            msg.force_torque = drc.force_torque_t();
        end
        
    end % end methods
end
