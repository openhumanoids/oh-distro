function init_lQb = init_orientation(mean_acc)

% orig_path = path();
% addpath([getenv('HOME'), '/drc/software/motion_estimate/DataFusion/matlab/INS']);
% addpath([getenv('HOME'), '/drc/software/motion_estimate/DataFusion/matlab/generic_functions']);
% 
% all_acc = cat(2,imu_data.pos_acc)';
% all_vel = cat(2,imu_data.rot_vel)';
% figure, plot(all_vel);
% mean_acc = mean(all_acc(1:1000,:));
clc
ab = mean_acc(:)/norm(mean_acc);

ab
% v2 = cross(v3,v1);

if (false)
    v1 = [0;0;1];
    v2 = [1;0;0];
    v3 = cross(v1,v2);
    v3m = cross(ab,v2);
    
    A = [v1,v2,v3];
    B = [ab(:)/norm(ab),v2(:)/norm(v2),v3m(:)/norm(v3m)];
    R_nav_to_body = B*inv(A) % synthetic heading alignment is not working
    
else
    roll = atan2(-ab(2),-ab(3))
    pitch = atan2(ab(1), norm(ab(2:3)))
    R_body_to_nav = q2R(e2q([roll;pitch;0]));
    
    % purposefully flip about x axis -- not part of the normal procedure
    % kept for future refenence
%     R_body_to_nav = q2R([0;1;0;0])*R_body_to_nav;
    R_nav_to_body = R_body_to_nav';
%     q_nb = R2q(R_nav_to_body);
end

R_nav_to_body'*mean_acc

init_lQb = R2q(R_nav_to_body)';
% qrot((init_lQb), mean_acc)



return

%% other method

pose_k1 = init_pose();
pose_k2 = init_pose();
pose_k1.utime = imu_data(1).utime;
%pose_k1.lQb = q_nav_to_body(:);
pose_k1.lQb = rot2quat(R_body_to_nav');

qrot(qconj(rot2quat(R_body_to_nav')),ab)
qrot(qconj(pose_k1.lQb),ab)

pose_k2.lQb = rot2quat(R_body_to_nav');
inertial_data = init_inertialData(9.8);

poses = cell(numel(imu_data),1);
ins_poses = cell(numel(imu_data),1);
for i = 1:numel(imu_data)

    % generate IMU data measurement frame
    inertial_data.predicted.utime = imu_data(i).utime;
    inertial_data.predicted.w_b = imu_data(i).rot_vel(:);
    inertial_data.predicted.a_b = imu_data(i).pos_acc(:) - 9.8*[0;0;-.0015];

    % get integrated pose
    ins_pose = INS_lQb([], pose_k1, pose_k2, inertial_data);
    ins_poses{i} = ins_pose;
    
    % convert pose data
    cur_pose.T = ins_pose.P_l(:)';
    cur_pose.R = q2R(ins_pose.lQb(:));
    poses{i} = cur_pose;

    % update history
    pose_k2 = pose_k1;
    pose_k1 = ins_pose;
end
poses = cell2mat(poses);
ins_poses = cell2mat(ins_poses);
figure, plot([ins_poses.f_l]'); title('f\_l');
figure, plot([ins_poses.a_l]'); title('a\_l');

path(orig_path);


function pose = init_pose()
pose.utime = 0;
pose.a_l = zeros(3,1);
pose.f_l = zeros(3,1);
pose.P_l = zeros(3,1);
pose.V_l = zeros(3,1);
pose.lQb = [1;0;0;0];
