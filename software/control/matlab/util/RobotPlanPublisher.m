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
            if nargin < 3
                utime = now() * 24 * 60 * 60;
            end                  
            switch nargin
                case 4
                    X = varargin{1};
                    T = varargin{2};
                    utime= varargin{3};
                    msg = encodeRobotPlan(obj,X,T,utime);
                    obj.lc.publish(obj.channel,msg);
                case 5
                    X = varargin{1};
                    T = varargin{2};
                    G = varargin{3};
                    utime= varargin{4};
                    msg = encodeRobotPlanWGraspTransitions(obj,X,T,G,utime);
                    obj.lc.publish(obj.channel, msg);
                otherwise
                    error('Incorrect usage of publish in RobotPlanPublisher. Undefined number of varargin.')
            end
        end


        function msg = encodeRobotPlan(obj,X,T,t)
            if nargin < 4
                t = now() * 24 * 60 * 60;
            end
            
            msg = drc.robot_plan_t();
            msg.utime = t * 1000000;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            offset= 0;
            num_dofs = (size(X,1)-offset)/2;
            plan = javaArray('drc.robot_state_t', msg.num_states);
            float_offset = 0;
            for i=1:msg.num_states,        
                is_keyframe(i) = (X(1,i)==1.0);
                is_breakpoint(i) = (X(2,i)==1.0);
                plan(i) = drc.robot_state_t();
                plan(i).utime = (T(i)*1000000);% use relative time, not absolute (G(i).utime*1000000+msg.utime)
                plan(i).robot_name = msg.robot_name;
                if obj.floating
                    plan(i).origin_position = drc.position_3d_t();
                    plan(i).origin_position.translation = drc.vector_3d_t();
                    plan(i).origin_position.rotation = drc.quaternion_t();
                    plan(i).origin_position.translation.x = X(3,i);
                    plan(i).origin_position.translation.y = X(4,i);
                    plan(i).origin_position.translation.z = X(5,i);
                    
                    q = rpy2quat([X(6,i) X(7,i) X(8,i)]);                    
                    plan(i).origin_position.rotation.w = q(1);
                    plan(i).origin_position.rotation.x = q(2);
                    plan(i).origin_position.rotation.y = q(3);
                    plan(i).origin_position.rotation.z = q(4);
                    
                    plan(i).origin_twist = drc.twist_t();
                    plan(i).origin_twist.linear_velocity = drc.vector_3d_t();
                    plan(i).origin_twist.angular_velocity = drc.vector_3d_t();   
                    plan(i).origin_twist.linear_velocity.x = X(offset+num_dofs+1,i);
                    plan(i).origin_twist.linear_velocity.y = X(offset+num_dofs+2,i);
                    plan(i).origin_twist.linear_velocity.z = X(offset+num_dofs+3,i);
                    plan(i).origin_twist.angular_velocity.x = X(offset+num_dofs+4,i);
                    plan(i).origin_twist.angular_velocity.y = X(offset+num_dofs+5,i);
                    plan(i).origin_twist.angular_velocity.z = X(offset+num_dofs+6,i);
                    
                    float_offset = 6;
                end
                plan(i).origin_cov =drc.covariance_t();
                plan(i).num_joints = num_dofs - float_offset;
                plan(i).joint_name=javaArray('java.lang.String', plan(i).num_joints);
                plan(i).joint_position=zeros(1,plan(i).num_joints);
                plan(i).joint_velocity=zeros(1,plan(i).num_joints);
                plan(i).measured_effort=zeros(1,plan(i).num_joints);
                plan(i).joint_cov= javaArray('drc.joint_covariance_t', plan(i).num_joints);
                for j=float_offset+1:num_dofs,
                    plan(i).joint_name(j-float_offset) = java.lang.String(obj.joint_names{j});
                    plan(i).joint_position(j-float_offset) = X(j+offset,i);
                    plan(i).joint_velocity(j-float_offset) = X(j+offset+num_dofs,i);
                    plan(i).joint_cov(j-float_offset) =drc.joint_covariance_t();
                end
 
                plan(i).contacts = drc.contact_state_t();
                plan(i).contacts.num_contacts=0;
            end
            msg.plan = plan;
            msg.num_grasp_transitions = 0;
            msg.num_bytes = 0;
            msg.arms_control_type = 0;
            msg.legs_control_type = 0;
        end
        
        function msg = encodeRobotPlanWGraspTransitions(obj,X,T,G,t) % G is grasp_transition_state structure
            if nargin < 5
                t = now() * 24 * 60 * 60;
            end
            msg = encodeRobotPlan(obj,X,T,t);
            
            % append grasp transition states to robot plan.
            msg.num_grasp_transitions = length([G.utime]);
            grasp_transition_states = javaArray('drc.grasp_transition_state_t', msg.num_grasp_transitions);
            for i=1:msg.num_grasp_transitions
              grasp_transition_states(i)= drc.grasp_transition_state_t();
              grasp_transition_states(i).utime = (G(i).utime*1000000);% use relative time, not absolute (G(i).utime*1000000+msg.utime)
              %grasp_transition_states(i).affordance_uid = 0;%(Ignore this field for now.)
              grasp_transition_states(i).affordance_uid =G(i).affordance_uid; 
              grasp_transition_states(i).hand_pose = drc.position_3d_t();
              grasp_transition_states(i).hand_pose.translation = drc.vector_3d_t();
              grasp_transition_states(i).hand_pose.rotation = drc.quaternion_t();
              grasp_transition_states(i).hand_pose.translation = G(i).hand_pose.translation;
              grasp_transition_states(i).hand_pose.rotation = G(i).hand_pose.rotation;
              grasp_transition_states(i).grasp_on = G(i).grasp_on;
              grasp_transition_states(i).grasp_type = G(i).grasp_type;
              grasp_transition_states(i).power_grasp = G(i).power_grasp;
              grasp_transition_states(i).num_joints = G(i).num_joints;
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
