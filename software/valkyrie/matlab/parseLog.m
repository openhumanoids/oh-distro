function main()

filename_root = '20141006_091705_Valkyrie_OK_Sidesteps'
%load('/home/mfallon/Desktop/val_data/20141017_DataForTwan/20141006_132958_Valkyrie_NiceRunOverCinderBlocks_NoArms.mat')
%load('/home/mfallon/Desktop/val_data/20141017_DataForTwan/20141005_153243_Valkyrie_TenNiceShortSteps.mat')


read_fromIHMCfile = false;
if (read_fromIHMCfile)
  % this takes 10mins
  root=readIHMCData(filename_root)
  disp('continue to save data')
  keyboard
  save root
else
  load([filename_root '.mat'])
end

publishToLCM(root)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function publishToLCM(root)
d = root.V1;

lc = lcm.lcm.LCM.getSingleton();

m = bot_core.pose_t();
s = drc.robot_state_t();

% WaistRotator	back_bkz	1
% WaistExtensor	back_bky	2
% WaistLateralExtensor	back_bkx	3
% LowerNeckExtensor	neck_ay	4
% NeckRotator	neck_by	5
% UpperNeckExtensor	neck_cy	6
% RightShoulderExtensor	r_arm_usy	7
% RightShoulderAdductor	r_arm_shx	8
% RightShoulderSupinator	r_arm_ely	9
% RightElbowExtensor	r_arm_elx	10
% RightForearmSupinator	r_arm_uwy	11
% RightWristExtensor	r_arm_mwx	12
% RightWrist	r_arm_lwz	13
% LeftShoulderExtensor	l_arm_usy	14
% LeftShoulderAdductor	l_arm_shx	15
% LeftShoulderSupinator	l_arm_ely	16
% LeftElbowExtensor	l_arm_elx	17
% LeftForearmSupinator	l_arm_uwy	18
% LeftWristExtensor	l_arm_mwx	19
% LeftWrist	l_arm_lwz	20
% RightHipRotator	r_leg_hpz	21
% RightHipAdductor	r_leg_hpx	22
% RightHipExtensor	r_leg_hpy	23
% RightKneeExtensor	r_leg_kny	24
% RightAnkleExtensor	r_leg_aky	25
% RightAnkle	r_leg_akx	26
% LeftHipRotator	l_leg_hpz	27
% LeftHipAdductor	l_leg_hpx	28
% LeftHipExtensor	l_leg_hpy	29
% LeftKneeExtensor	l_leg_kny	30
% LeftAnkleExtensor	l_leg_aky	31
% LeftAnkle	l_leg_akx	32

s.joint_name ={'back_bkz', 'back_bky', 'back_bkx', 'neck_ay', 'neck_by', 'neck_cy', ...
                 'r_arm_usy','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_uwy','r_arm_mwx','r_arm_lwz',  ...
                 'l_arm_usy','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','l_arm_lwz', ...
                 'r_leg_hpz','r_leg_hpx','r_leg_hpy','r_leg_kny','r_leg_aky','r_leg_akx', ...
                 'l_leg_hpz','l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx'}

names_in = {'WaistRotator', 'WaistExtensor', 'WaistLateralExtensor', 'LowerNeckExtensor', ...
            'NeckRotator', 'UpperNeckExtensor', 'RightShoulderExtensor', 'RightShoulderAdductor',...
            'RightShoulderSupinator', 'RightElbowExtensor', 'RightForearmSupinator', ...
            'RightWristExtensor', 'RightWrist', 'LeftShoulderExtensor', 'LeftShoulderAdductor', ...
            'LeftShoulderSupinator', 'LeftElbowExtensor', 'LeftForearmSupinator', ...
            'LeftWristExtensor', 'LeftWrist', 'RightHipRotator', 'RightHipAdductor', ...
            'RightHipExtensor', 'RightKneeExtensor', 'RightAnkleExtensor', 'RightAnkle', ...
            'LeftHipRotator', 'LeftHipAdductor', 'LeftHipExtensor', 'LeftKneeExtensor', ...
            'LeftAnkleExtensor', 'LeftAnkle'}

s.num_joints =size(s.joint_name,1);
s.joint_position = zeros(s.num_joints,1);
s.joint_velocity = zeros(s.num_joints,1);
s.joint_effort = zeros(s.num_joints,1);
pose = drc.position_3d_t();
pose.translation = drc.vector_3d_t();
pose.rotation = drc.quaternion_t();
twist = drc.twist_t();
twist.linear_velocity = drc.vector_3d_t();
twist.angular_velocity = drc.vector_3d_t();
s.twist = twist;
s.pose = pose;
s.force_torque=drc.force_torque_t();

for i=2:1:size(d.q_z,2)
  %m.utime = d.t(i)*1E6;
  %m.pos = [d.q_x(i), d.q_y(i), d.q_z(i)];
  %m.orientation = [d.q_qs(i),d.q_qx(i), d.q_qy(i), d.q_qz(i)];
  %lc.publish('POSE_BODY', m);
  
  s.utime = d.t(i)*1E6;
  for j=1:size(names_in,2)
     s.joint_position(j) = d.(['q_' names_in{j}])(i);
     %s.joint_velocity(j) = d.(['qd_' names_in{j}])(i);
     %s.joint_velocity(j) = d.('qd_LeftHipRotator')(i);
     %s.joint_effort(j) = d.(['qdd_' names_in{j}])(i);     
  end
  
  s.pose.translation.x = d.q_x(i);
  s.pose.translation.y = d.q_y(i);
  s.pose.translation.z = d.q_z(i);
  s.pose.rotation.w = d.q_qs(i);
  s.pose.rotation.x = d.q_qx(i);
  s.pose.rotation.y = d.q_qy(i);
  s.pose.rotation.z = d.q_qz(i);  
  lc.publish('EST_ROBOT_STATE', s);
  %pause(0.001)  
end


function root=readIHMCData(filename_root)

%fid=fopen('20141006_091705_Valkyrie_OK_Sidesteps.data');
%fid=fopen('20141006_132958_Valkyrie_NiceRunOverCinderBlocks_NoArms.data');
fid=fopen([filename_root '.data']);
% 10476 fields exist
i=0
while 1
    tline = fgetl(fid);
    if ~ischar(tline), break, end
    %disp(tline);
    disp(i)
    tline = strrep(tline,'Infinity','Inf');
    eval(tline)
    
    i=i+1;
end
fclose(fid);