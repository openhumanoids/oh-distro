classdef RobotPlanPublisherWKeyFrames
    properties
        lc;
        channel;
        floating;
        joint_names;
    end
    
    methods

        function obj = RobotPlanPublisherWKeyFrames(channel,floating,joint_names)
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
                case 6
                    X = varargin{1};
                    T = varargin{2};
                    utime= varargin{3};
                    snopt_info_vector = varargin{4};
                    G = varargin{5};                    
                    msg = encodeRobotPlanWGraspTransitions(obj,X,T,utime,snopt_info_vector,G);
                    obj.lc.publish(obj.channel, msg);
                otherwise
                    error('Incorrect usage of publish in RobotPlanPublisherWKeyFrames. Undefined number of varargin.')
            end
        end


        function msg = encodeRobotPlan(obj,X,T,t, snopt_info_vector)
            if nargin < 5
                snopt_info_vector = zeros(1,size(X,2));
            end
            if nargin < 4
                t = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            
            
            
            msg = drc.robot_plan_w_keyframes_t();
            msg.utime = t;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            msg.num_keyframes = sum(X(1,:));
            msg.num_breakpoints = sum(X(2,:));
            offset= 2;
            num_dofs = (size(X,1)-offset)/2;
            is_keyframe = logical(zeros(1,msg.num_states)==1); 
            %javaArray('java.lang.Boolean', msg.num_states);
            is_breakpoint = logical(zeros(1,msg.num_states)==1);
            %javaArray('java.lang.Boolean', msg.num_states);
            plan = javaArray('bot_core.robot_state_t', msg.num_states);
            float_offset = 0;
            for i=1:msg.num_states,        
                is_keyframe(i) = (X(1,i)==1.0);
                is_breakpoint(i) = (X(2,i)==1.0);
                plan(i) = bot_core.robot_state_t();
                plan(i).utime = (T(i)*1000000);% use relative time //
                if obj.floating
                    plan(i).pose = bot_core.position_3d_t();
                    plan(i).pose.translation = bot_core.vector_3d_t();
                    plan(i).pose.rotation = bot_core.quaternion_t();
                    plan(i).pose.translation.x = X(3,i);
                    plan(i).pose.translation.y = X(4,i);
                    plan(i).pose.translation.z = X(5,i);
                    
                    q = rpy2quat([X(6,i) X(7,i) X(8,i)]);                    
                    plan(i).pose.rotation.w = q(1);
                    plan(i).pose.rotation.x = q(2);
                    plan(i).pose.rotation.y = q(3);
                    plan(i).pose.rotation.z = q(4);
                    
                    plan(i).twist = bot_core.twist_t();
                    plan(i).twist.linear_velocity = bot_core.vector_3d_t();
                    plan(i).twist.angular_velocity = bot_core.vector_3d_t();   
                    plan(i).twist.linear_velocity.x = X(offset+num_dofs+1,i);
                    plan(i).twist.linear_velocity.y = X(offset+num_dofs+2,i);
                    plan(i).twist.linear_velocity.z = X(offset+num_dofs+3,i);
                    plan(i).twist.angular_velocity.x = X(offset+num_dofs+4,i);
                    plan(i).twist.angular_velocity.y = X(offset+num_dofs+5,i);
                    plan(i).twist.angular_velocity.z = X(offset+num_dofs+6,i);
                    
                    float_offset = 6;
                end
                plan(i).num_joints = num_dofs - float_offset;
                plan(i).joint_name=javaArray('java.lang.String', plan(i).num_joints);
                plan(i).joint_position=zeros(1,plan(i).num_joints);
                plan(i).joint_velocity=zeros(1,plan(i).num_joints);
                plan(i).joint_effort=zeros(1,plan(i).num_joints);
                for j=float_offset+1:num_dofs,
                    plan(i).joint_name(j-float_offset) = java.lang.String(obj.joint_names{j});
                    plan(i).joint_position(j-float_offset) = X(j+offset,i);
                    plan(i).joint_velocity(j-float_offset) = X(j+offset+num_dofs,i);
                end
 
                plan(i).force_torque = bot_core.force_torque_t();
            end
            msg.is_keyframe = is_keyframe;
            msg.is_breakpoint = is_breakpoint;
            msg.plan = plan;
            msg.plan_info = snopt_info_vector;
            msg.num_grasp_transitions = 0;
            msg.num_bytes = 0;
        end
        
        function msg = encodeRobotPlanWGraspTransitions(obj,X,T,t, snopt_info_vector,G) % G is grasp_transition_state structure

            msg = encodeRobotPlan(obj,X,T,t, snopt_info_vector);
            
            % append grasp transition states to robot plan.
            msg.num_grasp_transitions = length([G.utime]);
            grasp_transition_states = javaArray('drc.grasp_transition_state_t', msg.num_grasp_transitions);
            for i=1:msg.num_grasp_transitions
              grasp_transition_states(i)= drc.grasp_transition_state_t();
              grasp_transition_states(i).utime = (G(i).utime*1000000);% use relative time //+msg.utime
              %grasp_transition_states(i).affordance_uid = 0;%(Ignore this field for now.)
              grasp_transition_states(i).affordance_uid =G(i).affordance_uid; 
              grasp_transition_states(i).hand_pose = bot_core.position_3d_t();
              grasp_transition_states(i).hand_pose.translation = bot_core.vector_3d_t();
              grasp_transition_states(i).hand_pose.rotation = bot_core.quaternion_t();
              grasp_transition_states(i).hand_pose.translation = G(i).hand_pose.translation;
              grasp_transition_states(i).hand_pose.rotation = G(i).hand_pose.rotation;
              grasp_transition_states(i).grasp_on = G(i).grasp_on;
              grasp_transition_states(i).grasp_type = G(i).grasp_type;
              grasp_transition_states(i).power_grasp = G(i).power_grasp;
              grasp_transition_states(i).num_joints = (G(i).num_joints);
              grasp_transition_states(i).joint_name=javaArray('java.lang.String', G(i).num_joints);
              grasp_transition_states(i).joint_position=zeros(1,G(i).num_joints);
              for j=1:G(i).num_joints
                  grasp_transition_states(i).joint_name(j) = G(i).joint_name(j);
                  grasp_transition_states(i).joint_position(j) =G(i).joint_position(j);
              end
            end
            msg.grasps =grasp_transition_states;
           
        end        
        
    end % end methods
end
