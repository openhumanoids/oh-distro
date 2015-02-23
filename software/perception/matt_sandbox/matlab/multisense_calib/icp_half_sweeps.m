function result = icp_half_sweeps(scans1,scans2,res_in,iters,frac)

dist_thresh = 0.1;
optimize_worst = false;
index_cell_size = 0.1;

poses_start1 = cat(1,scans1.pose_start);
poses_end1 = cat(1,scans1.pose_end);
poses_start2 = cat(1,scans2.pose_start);
poses_end2 = cat(1,scans2.pose_end);

range_range = [2,10];
theta_range = [-360,360];
range_filter_thresh = 40;

[~,data1] = accum_scans(scans1,res_in.P_pre_spindle_to_camera,res_in.P_lidar_to_post_spindle,...
    range_range, theta_range, range_filter_thresh);
[~,data2] = accum_scans(scans2,res_in.P_pre_spindle_to_camera,res_in.P_lidar_to_post_spindle,...
    range_range, theta_range, range_filter_thresh);
data1 = sortrows(data1,5);
data2 = sortrows(data2,5);

res = res_in;
for iter = 1:iters
    fprintf('starting iter %d/%d...\n', iter, iters);

    % accumulate current points
    pts1 = accum_lidar(data1,poses_start1,poses_end1,...
        res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,true);
    pts2 = accum_lidar(data2,poses_start2,poses_end2,...
        res.P_pre_spindle_to_camera,res.P_lidar_to_post_spindle,true);

    % create index and map points into it
    index1 = build_spatial_index(pts1(:,1:3),index_cell_size,1);
    bins2 = round([pts2(:,1:3),ones(size(pts2,1),1)]*index1.xform(1:3,:)');
    sz = size(index1.vox);
    good = all(bins2>=1,2) & all(bins2<=repmat(sz([2,1,3]),[size(bins2,1),1]),2);
    pts2_good = pts2(good,:);
    inds2 = sub2ind(sz,bins2(good,2),bins2(good,1),bins2(good,3));
    data2_good = data2(good,:);
    fprintf('created index\n');
    
    % create matches
    matches = {};
    for i = 1:numel(inds2)
        idx = index1.vox{inds2(i)};
        p1 = pts1(idx,:);
        p2 = pts2_good(i,:);
        d = [p1(:,1)-p2(1),p1(:,2)-p2(2),p1(:,3)-p2(3)];
        d2 = d(:,1).^2 + d(:,2).^2 + d(:,3).^2;
        [minval,minidx] = min(d2);
        if (minval < dist_thresh^2)
            matches{end+1,1} = [idx(minidx),i];
        end
        if (mod(i,50000)==0)
            fprintf('%d/%d\n', i, numel(inds2));
        end
    end
    matches = cell2mat(matches);
    fprintf('found %d matches\n', size(matches,1));
    if (optimize_worst)
        p1 = pts1(matches(:,1),:);
        p2 = pts2_good(matches(:,2),:);
        d = sqrt(sum((p2-p1).^2,2));
        [~,sort_ind] = sort(d,'descend');
        matches = matches(sort_ind(1:ceil(numel(d)*frac)),:);
    else
        subsamp = ceil(1/frac);
        matches = matches(1:subsamp:end,:);
    end
    fprintf('  (using %d matches)\n', size(matches,1));

    res_new = optimize_spindle_pose(data1(matches(:,1),:),data2_good(matches(:,2),:),...
        poses_start1, poses_end1, poses_start2, poses_end2, ...
        res.P_pre_spindle_to_camera, res.P_lidar_to_post_spindle,false);
    res = res_new;
end
result = res;
