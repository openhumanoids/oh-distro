function pts = accum_lidar(data, poses_start, poses_end,...
    P_camera_to_pre_spindle,P_post_spindle_to_lidar,is_sorted)

% data row format: range, theta, x, y, scan id, point percent, extra....

if (~exist('is_sorted','var'))
    is_sorted = false;
end

do_interp = (numel(poses_start)==numel(poses_end));

if (~is_sorted)
    [data,sort_ind] = sortrows(data,5);
end

d = diff(data(:,5));
starts = [1;find(d>0)+1];
ends = [starts(2:end)-1;numel(d)+1];
pts = zeros(size(data,1),3);
counter = 1;
for i = 1:numel(starts)
    data_sub = data(starts(i):ends(i),:);
    scan_ind = data_sub(1,5);

    % compute poses
    if (do_interp)
        P_pre_spindle_to_post_spindle_start = [poses_start(scan_ind).R,poses_start(scan_ind).T(:);0,0,0,1];
        P_start = inv(P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle_start*P_camera_to_pre_spindle);
    end
    P_pre_spindle_to_post_spindle_end = [poses_end(scan_ind).R,poses_end(scan_ind).T(:);0,0,0,1];
    P_end = inv(P_post_spindle_to_lidar*P_pre_spindle_to_post_spindle_end*P_camera_to_pre_spindle);

    if (do_interp)
        % interpolate poses
        alphas = data_sub(:,6);
        T_interp = (1-alphas(:))*P_start(1:3,4)' + alphas(:)*P_end(1:3,4)';
        q_start = rot2quat(P_start(1:3,1:3));
        q_end = rot2quat(P_end(1:3,1:3));
        angle = acos(dot(q_start,q_end));
        q_interp = (sin((1-alphas(:))*angle)*q_start(:)' + sin(alphas(:)*angle)*q_end(:)')/sin(angle);
        R_interp = quat2rot_vectorized(q_interp);
        Rx_interp = R_interp(:,[1,4,7]);
        Ry_interp = R_interp(:,[2,5,8]);
        p = data_sub(:,[3,3,3]).*Rx_interp + data_sub(:,[4,4,4]).*Ry_interp + T_interp;
    else
        % just use the end pose for this scan
        R = P_end(1:3,1:3);
        T = P_end(1:3,4);
        p = data_sub(:,3:4)*R(:,1:2)';
        p = [p(:,1)+T(1), p(:,2)+T(2), p(:,3)+T(3)];
    end

    sz = size(data_sub,1);
    pts(counter:counter+sz-1,:) = p;
    counter = counter+sz;
end

% restore original order
if (~is_sorted)
    pts(sort_ind,:) = pts;
end
