function publishERS(robot_state, joints_names, channel_name)
%
%robot_state = [0.0;0.0;2.025;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.0;0.30000001192092896;-0.20000000298023224;0.0;-0.800000011920929;0.0;0.0;0.0;-0.25132742524147034;-0.20000000298023224;0.0;-1.293114423751831;-0.14608405530452728;0.0;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0;0.0;0.0;-0.49000000953674316;-1.2200000286102295;-0.7200000286102295;0.0];
%joints_names = {'base_x', 'base_y', 'base_z', 'base_roll', 'base_pitch', 'base_yaw', 'WaistRotator', 'WaistExtensor', 'WaistLateralExtensor', 'LowerNeckExtensor', 'NeckRotator', 'UpperNeckExtensor', 'RightShoulderExtensor', 'RightShoulderAdductor', 'RightShoulderSupinator', 'RightElbowExtensor', 'RightForearmSupinator', 'RightWristExtensor', 'RightWrist', 'LeftShoulderExtensor', 'LeftShoulderAdductor', 'LeftShoulderSupinator', 'LeftElbowExtensor', 'LeftForearmSupinator', 'LeftWristExtensor', 'LeftWrist', 'LeftHipRotator', 'LeftHipAdductor', 'LeftHipExtensor', 'LeftKneeExtensor', 'LeftAnkleExtensor', 'LeftAnkle', 'RightHipRotator', 'RightHipAdductor', 'RightHipExtensor', 'RightKneeExtensor', 'RightAnkleExtensor', 'RightAnkle'};
%channel_name = "EST_ROBOT_STATE"
%publishERS(robot_state, joints_names, channel_name)

state_msg = drc.robot_state_t();

num_dofs = size(robot_state,2);
      
state_msg.utime = 0;

% ATLAS_STATE has no global pose, but EST_ROBOT_STATE does
state_msg.pose = drc.position_3d_t();
state_msg.pose.translation = drc.vector_3d_t();
state_msg.pose.rotation = drc.quaternion_t();
state_msg.pose.translation.x = robot_state(1);
state_msg.pose.translation.y = robot_state(2);
state_msg.pose.translation.z = robot_state(3);

yaw = robot_state(6);
yaw = mod(yaw, 2*pi);
if (yaw > pi)
    yaw = yaw - 2*pi;
end
q = rpy2quat([robot_state(4) robot_state(5) yaw]);
state_msg.pose.rotation.w = q(1);
state_msg.pose.rotation.x = q(2);
state_msg.pose.rotation.y = q(3);
state_msg.pose.rotation.z = q(4);

state_msg.twist = drc.twist_t();
state_msg.twist.linear_velocity = drc.vector_3d_t();
state_msg.twist.angular_velocity = drc.vector_3d_t();
%state_msg.twist.linear_velocity.x = 0;%robot_state(atlas_dofs+1);
%state_msg.twist.linear_velocity.y = 0;%robot_state(atlas_dofs+2);
%state_msg.twist.linear_velocity.z = 0;%robot_state(atlas_dofs+3);
%avel = rpydot2angularvel(robot_state(4:6), robot_state(atlas_dofs+4:atlas_dofs+6));
%state_msg.twist.angular_velocity.x = avel(1);
%state_msg.twist.angular_velocity.y = avel(2);
%state_msg.twist.angular_velocity.z = avel(3);

% will publish the multisense angle as part of total
% est_robot_state
state_msg.num_joints = num_dofs-6;
% only est_robot_state robot_state message has joint names
state_msg.joint_name = joints_names(6+1:end);


state_msg.joint_position=zeros(1,state_msg.num_joints);
state_msg.joint_velocity=zeros(1,state_msg.num_joints);
state_msg.joint_effort=zeros(1,state_msg.num_joints);

atlas_pos = robot_state(7:end);
%atlas_vel = robot_state(atlas_dofs+7:end);

state_msg.joint_position = atlas_pos ;%[atlas_pos; right_hand_state(1:right_hand_dofs); left_hand_state(1:left_hand_dofs); laser_spindle_angle];
state_msg.joint_velocity = atlas_pos ;%[atlas_vel; right_hand_state(right_hand_dofs+1:end); left_hand_state(left_hand_dofs+1:end); 0];
state_msg.force_torque = drc.force_torque_t();

obj.lc = lcm.lcm.LCM.getSingleton();
obj.lc.publish(channel_name, state_msg);
