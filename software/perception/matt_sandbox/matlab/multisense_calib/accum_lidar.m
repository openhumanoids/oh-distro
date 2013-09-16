function pts = accum_lidar(data,poses,P_camera_to_pre_spindle,P_post_spindle_to_lidar)

[data,sort_ind] = sortrows(data,4);
d = diff(data(:,4));
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];
pts = zeros(size(data,1),3);
counter = 1;
for i = 1:numel(starts)
    data_sub = data(starts(i):ends(i),:);
    scan_ind = data_sub(1,4);
    P_pre_spindle_to_post_spindle = [poses(scan_ind).R,poses(scan_ind).T(:);0,0,0,1];
    P = P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle*P_camera_to_pre_spindle;
    P(1:3,1:3) = P(1:3,1:3)';
    P(1:3,4) = -P(1:3,1:3)*P(1:3,4);
    sz = size(data_sub,1);
    pts(counter:counter+sz-1,:) = [data_sub(:,1:3),ones(size(data_sub,1),1)]*P(1:3,:)';
    counter = counter+sz;
end
pts(sort_ind,:) = pts;
