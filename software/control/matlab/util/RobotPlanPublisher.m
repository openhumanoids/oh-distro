classdef RobotPlanPublisher
    properties
        lc;
        channel;
        floating;
        joint_names;
    end
    
    methods

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

        function publish(obj,varargin)
            if nargin < 4
               utime = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            switch nargin
                case 4
                    X = varargin{1};
                    T = varargin{2};
                    utime= varargin{3};
                    snopt_info_vector = zeros(1,size(X,2));
                    msg = encodeRobotPlan(obj,X,T,utime,snopt_info_vector);
                    obj.lc.publish(obj.channel,msg);
                case 5
                    X = varargin{1};
                    T = varargin{2};
                    utime= varargin{3};
                    snopt_info_vector = varargin{4};
                    msg = encodeRobotPlan(obj,X,T,utime,snopt_info_vector);
                    obj.lc.publish(obj.channel,msg);
                otherwise
                    error('Incorrect usage of publish in RobotPlanPublisher. Undefined number of varargin.')
            end
        end


        function msg = encodeRobotPlan(obj,X,T,t, snopt_info_vector)
            if nargin < 5
                snopt_info_vector = zeros(1,size(X,2));
            end
            if nargin < 4
                t = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            
            
            
            msg = drc.robot_plan_t();
            msg.utime = t;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            num_dofs = size(X,1)/2;
            plan = javaArray('drc.robot_state_t', msg.num_states);
            float_offset = 0;
            for i=1:msg.num_states,        
                plan(i) = drc.robot_state_t();
                plan(i).utime = (T(i)*1000000);% use relative time //
                if obj.floating
                    plan(i).pose = drc.position_3d_t();
                    plan(i).pose.translation = drc.vector_3d_t();
                    plan(i).pose.rotation = drc.quaternion_t();
                    plan(i).pose.translation.x = X(1,i);
                    plan(i).pose.translation.y = X(2,i);
                    plan(i).pose.translation.z = X(3,i);
                    
                    q = rpy2quat([X(4,i) X(5,i) X(6,i)]);                    
                    plan(i).pose.rotation.w = q(1);
                    plan(i).pose.rotation.x = q(2);
                    plan(i).pose.rotation.y = q(3);
                    plan(i).pose.rotation.z = q(4);
                    
                    plan(i).twist = drc.twist_t();
                    plan(i).twist.linear_velocity = drc.vector_3d_t();
                    plan(i).twist.angular_velocity = drc.vector_3d_t();   
                    plan(i).twist.linear_velocity.x = X(num_dofs+1,i);
                    plan(i).twist.linear_velocity.y = X(num_dofs+2,i);
                    plan(i).twist.linear_velocity.z = X(num_dofs+3,i);
                    plan(i).twist.angular_velocity.x = X(num_dofs+4,i);
                    plan(i).twist.angular_velocity.y = X(num_dofs+5,i);
                    plan(i).twist.angular_velocity.z = X(num_dofs+6,i);
                    
                    float_offset = 6;
                end
                plan(i).num_joints = num_dofs - float_offset;
                plan(i).joint_name=javaArray('java.lang.String', plan(i).num_joints);
                plan(i).joint_position=zeros(1,plan(i).num_joints);
                plan(i).joint_velocity=zeros(1,plan(i).num_joints);
                plan(i).joint_effort=zeros(1,plan(i).num_joints);
                for j=float_offset+1:num_dofs,
                    plan(i).joint_name(j-float_offset) = java.lang.String(obj.joint_names{j});
                    plan(i).joint_position(j-float_offset) = X(j,i);
                    plan(i).joint_velocity(j-float_offset) = X(j+num_dofs,i);
                end
 
                plan(i).force_torque = drc.force_torque_t();
            end
            msg.plan = plan;
            msg.plan_info = snopt_info_vector;
            msg.num_grasp_transitions = 0;
            msg.num_bytes = 0;
        end 
    end % end methods
end
