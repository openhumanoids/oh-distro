%%
logfile = '/home/antone/data/logs/lcmlog-2013-08-14.00_linear_rotation';
setup_lcm

%%
lcmlog = lcm.logging.Log(logfile,'r');
clear imu_poses;
imu_counter = 1;
vicon_counter = 1;
while (true)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = event.channel;
    if (strcmp(channel,'POSE_BDI'))
        pose = decode_lcm_pose(bot_core.pose_t(event.data));
        imu_poses(imu_counter) = pose;
        imu_counter = imu_counter+1;
    elseif (strcmp(channel,'VICON_ATLAS'))
        obj = bot_core.rigid_transform_t(event.data);
        pose.T = obj.trans;
        pose.R = quat2rot(obj.quat);
        pose.timestamp = int64(obj.utime);
        vicon_poses(vicon_counter) = pose;
        vicon_counter = vicon_counter+1;
    end
end

%% interpolate imu times to match vicon times
imu_times = cat(1,imu_poses.timestamp);
vicon_times = cat(1,vicon_poses.timestamp);
clear interp_poses;
for i = 1:numel(vicon_times)
    ind = find(imu_times<vicon_times(i),1,'last');
    if (numel(ind)==0 || ind>=numel(imu_poses))
        continue;
    end
    interp_poses(i) = interpolate_pose(imu_poses(ind), imu_poses(ind+1), vicon_times(i));
    interp_times(i) = imu_poses(ind).timestamp;
    %fprintf('%d %d %d\n', imu_poses(ind).timestamp, imu_poses(ind+1).timestamp, vicon_times(i));
end

%%
figure
for i = 1:numel(vicon_poses)
    clf
    hold on;
    plot_pose(vicon_poses(i).R, [0;0;0], 1);
    plot_pose(interp_poses(i).R, [0;0;0], 1);
    hold off;
    axis equal;
    drawnow;
    if (i==1)
        view3d;
    end
end

%%
result = optimize_rotation(vicon_poses, interp_poses);

%%
figure;
for i = 1:10:numel(vicon_poses)
    R1 = vicon_poses(i).R;
    R2 = interp_poses(i).R*result.R';
    dR = R2'*R1;
    %hold on; plot_pose(dR,[0;0;0],1,''); hold off;
    clf;
    hold on; plot_pose(R1,[0;0;0],1,'');
    plot_pose(R2,[1;1;1],1,'');
    hold off;
    axis equal;
    if (i == 1)
        view3d
    end
    drawnow;
end
% axis equal;
% view3d



%%
clear pose_diffs;
for i = 1:numel(vicon_poses)-100
    dR1 = vicon_poses(i+100).R'*vicon_poses(i).R;
    dR2 = interp_poses(i+100).R'*interp_poses(i).R;
    pose_diffs(i).R = dR1'*dR2;
end

%%
figure, hold on;
for i = 1:10:numel(pose_diffs)
    plot_pose(pose_diffs(i).R, [0;0;0], 1, '');
end
hold off; axis equal; view3d




%% time tests
%%
logfile = '/home/antone/data/logs/lcmlog-2013-08-14.02_walking';
setup_lcm

%%
clear lcmlog
lcmlog = lcm.logging.Log(logfile,'r');
clear times;
counter = 1;
local_utimes = [];
wall_utimes = [];
bdi_utimes = [];
while (true)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = event.channel;
    if (strcmp(channel,'BDI_UTIMES'))
        obj = drc.behavior_command_t(event.data);
        str = char(obj.command);
        substrs = textscan(str,'%s','Delimiter',',');
        substrs = substrs{1};
        local_utime = str2double(substrs{2});
        wall_utime = str2double(substrs{4});
        bdi_utime = str2double(substrs{6});
        local_utimes = [local_utimes;local_utime];
        wall_utimes = [wall_utimes; wall_utime];
        bdi_utimes = [bdi_utimes; bdi_utime];
    end
end
