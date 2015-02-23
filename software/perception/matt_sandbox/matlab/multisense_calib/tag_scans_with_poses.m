function scans = tag_scans_with_poses(scans, poses, scan_time_us)
poses_end = synchronize_poses(poses, cat(1,scans.timestamp));
poses_start = synchronize_poses(poses, cat(1,scans.timestamp)-scan_time_us);
for i = 1:numel(scans)
    scans(i).pose_start = poses_start(i);
    scans(i).pose_end = poses_end(i);
end
