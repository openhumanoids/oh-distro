classdef AffIndexedRobotPlanPublisher
    
    properties
        lc;
        channel;
        floating;
        joint_names;
    end
    
    methods
        
        function obj = AffIndexedRobotPlanPublisher(channel,floating,joint_names)
            obj.channel = channel;
            obj.lc = lcm.lcm.LCM.getSingleton();
            if nargin > 1
                obj.floating = floating;
            else
                obj.floating = true;
            end
            obj.joint_names = joint_names;
        end
        
        function publish(obj, X, I, utime)
            if nargin < 3
                utime = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            msg=AffIndexedRobotPlanPublisher.encodeAffIndexedRobotPlan(obj,X,I,utime);
            %msg=AffIndexedRobotPlanPublisher.testrobotstate(); % WORKS
            %msg=AffIndexedRobotPlanPublisher.testrobotplan();% WORKS
            obj.lc.publish(obj.channel,msg);
        end
        
    end
    
    methods(Static)
        
        function msg=testrobotstate()
            % basic skeleton to get drc_robot_state_t to work
            msg = drc.robot_state_t();
            msg.utime = 0;
            msg.robot_name = 'atlas';
            msg.pose = drc.position_3d_t();
            msg.pose.translation = drc.vector_3d_t();
            msg.pose.rotation = drc.quaternion_t();
            msg.twist = drc.twist_t();
            msg.twist.linear_velocity = drc.vector_3d_t();
            msg.twist.angular_velocity = drc.vector_3d_t();
            msg.num_joints=2;
            
            msg.joint_name=javaArray('java.lang.String', msg.num_joints);
            msg.joint_position=zeros(1,msg.num_joints);
            msg.joint_velocity=zeros(1,msg.num_joints);
            msg.joint_effort=zeros(1,msg.num_joints);
            for j=1:msg.num_joints,
                msg.joint_name(j) = java.lang.String('hello');
            end
            
            msg.contacts = drc.contact_state_t();
            msg.contacts.num_contacts=0;
            
        end
        
        
        function msg=testrobotplan()
            msg = drc.robot_plan_t();
            msg.utime = 0;
            msg.robot_name = 'atlas';
            msg.num_states=2;
            
            msg.plan = javaArray('drc.robot_state_t', msg.num_states);
%             for i=1:msg.num_states
%                 msg.plan(i) = drc.robot_state_t();
%                 msg.plan(i) =AffIndexedRobotPlanPublisher.testrobotstate();
%             end
            for i=1:msg.num_states
                msg.plan(i) = drc.robot_state_t();
                msg.plan(i).utime = 0;
                msg.plan(i).robot_name = 'atlas';
                msg.plan(i).pose = drc.position_3d_t();
                msg.plan(i).pose.translation = drc.vector_3d_t();
                msg.plan(i).pose.rotation = drc.quaternion_t();
                msg.plan(i).twist = drc.twist_t();
                msg.plan(i).twist.linear_velocity = drc.vector_3d_t();
                msg.plan(i).twist.angular_velocity = drc.vector_3d_t();
                msg.plan(i).num_joints=2;
                
                msg.plan(i).joint_name=javaArray('java.lang.String', msg.plan(i).num_joints);
                msg.plan(i).joint_position=zeros(1,msg.plan(i).num_joints);
                msg.plan(i).joint_velocity=zeros(1,msg.plan(i).num_joints);
                msg.plan(i).joint_effort=zeros(1,msg.plan(i).num_joints);
                for j=1:msg.plan(i).num_joints,
                    msg.plan(i).joint_name(j) = java.lang.String('hello');
                end
                
                msg.plan_info = zeros(1, num_states);
                
                msg.plan(i).contacts = drc.contact_state_t();
                msg.plan(i).contacts.num_contacts=0;
            end
            msg.num_grasp_transitions = 0;
            msg.num_bytes = 0;
            msg.arms_control_type = 0;
            msg.legs_control_type = 0;
            
        end
        
        
        function msg = encodeAffIndexedRobotPlan(obj,X,I,t, snopt_info_vector)
            if nargin < 4
                snopt_info_vector = zeros(1,size(X,2));
            end          
            if nargin < 3
                t = get_timestamp_now();% equivalent to bot_timestamp_now();
            end
            
            
            msg = drc.aff_indexed_robot_plan_t();
            msg.utime = t;
            msg.robot_name = 'atlas';
            msg.num_states = size(X,2);
            num_dofs = size(X,1)/2;
            
            %plan = zeros(msg.num_states,1);
            plan = javaArray('drc.robot_state_t', msg.num_states);
            %aff_index = javaArray('drc.affordance_index_t', msg.num_states);
            %aff_index = zeros(msg.num_states,1);
            float_offset = 0;
            for i=1:msg.num_states
                plan(i) = drc.robot_state_t();
                plan(i).utime = (I(i).time*1000000+msg.utime);
                plan(i).robot_name = msg.robot_name;
                aff_index(i)=drc.affordance_index_t();
                aff_index(i).utime = (I(i).time*1000000+msg.utime);
                aff_index(i).aff_type= I(i).aff_type;
                aff_index(i).aff_uid= I(i).aff_uid;
                aff_index(i).num_ees= I(i).num_ees;
                aff_index(i).ee_name=javaArray('java.lang.String', aff_index(i).num_ees);
                aff_index(i).dof_name=javaArray('java.lang.String', aff_index(i).num_ees);
                aff_index(i).dof_value=zeros(1,aff_index(i).num_ees);
                if(aff_index(i).num_ees>1)
                    for j=1:aff_index(i).num_ees,
                        aff_index(i).ee_name(j)  = I(i).ee_name(j);
                        aff_index(i).dof_name(j) = I(i).dof_name(j);
                        aff_index(i).dof_value(j)= I(i).dof_value(j);
                    end
                else
                    aff_index(i).ee_name(1)  = (I(i).ee_name);
                    aff_index(i).dof_name(1) = (I(i).dof_name);
                    aff_index(i).dof_value(1)= I(i).dof_value(1);      
                
                end
                
                if (obj.floating)
                    plan(i).pose = drc.position_3d_t();
                    plan(i).pose.translation = drc.vector_3d_t();
                    plan(i).pose.rotation = drc.quaternion_t();
                    plan(i).pose.translation.x = X(1,i);
                    plan(i).pose.translation.y = X(2,i);
                    plan(i).pose.translation.z = X(3,i);
                    
                    q = rpy2quat([X(4,i) X(5,i) X(6,i)]);
                    % q = angle2quat(X(4,i), X(5,i), X(6,i),'XYZ');
                    
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
                
                plan(i).contacts = drc.contact_state_t();
                plan(i).contacts.num_contacts=0;
                % plan(i).contacts.id=javaArray('java.lang.String', plan(i).contacts.num_contacts);
                % plan(i).contacts.contact_torque = javaArray('drc.vector_3d_t', plan(i).contacts.num_contacts);
                % plan(i).contacts.contact_force  = javaArray('drc.vector_3d_t', plan(i).contacts.num_contacts);
            end
            msg.aff_index=aff_index;
            msg.plan = plan;
            msg.plan_info = snopt_info_vector;
            msg.num_grasp_transitions = 0;
            msg.num_bytes = 0;
            msg.arms_control_type = 0;
            msg.legs_control_type = 0;
            %  msg.matlab_data=javaArray('java.lang.Byte',msg.num_bytes);
        end
    end
end


