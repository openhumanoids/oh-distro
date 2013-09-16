function poses = synchronize_poses(poses_in,new_times)

pose_times = cat(1,poses_in.timestamp);
ind = 1;
for i = 1:numel(new_times)
    while ((ind < numel(pose_times)) && (pose_times(ind+1) <= new_times(i)))
        ind = ind+1;
    end
    if (ind == numel(pose_times))
        ind = ind-1;
    end
    poses(i) = interpolate_pose(poses_in(ind),poses_in(ind+1),new_times(i));
end
