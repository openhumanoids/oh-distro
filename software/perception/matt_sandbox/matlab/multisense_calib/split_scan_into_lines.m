function segs = split_scan_into_lines(scan, range_range, theta_range, max_dist, min_pts)

thetas = scan.theta_min + scan.theta_step*(0:numel(scan.ranges)-1);
thetas = thetas(:);
bad_ind = scan.ranges<range_range(1) | scan.ranges>range_range(2);
bad_ind = bad_ind | thetas<theta_range(1) | thetas>theta_range(2);
xy = [cos(thetas(:)).*scan.ranges, sin(thetas(:)).*scan.ranges];
n = numel(scan.ranges);

cur_start = 1;
segs = {};
while(cur_start <= n)
    while((cur_start<=n) && bad_ind(cur_start))
        cur_start = cur_start+1;
    end
    cur_end = cur_start;

    while (true)
        test_end = cur_end+1;
        if ((test_end>n) || bad_ind(test_end))
            break;
        end
        pts = xy(cur_start:test_end,:);
        L = fit_line(pts);
        %         L = [xy(cur_start,2)-xy(test_end,2), xy(test_end,1)-xy(cur_start,1),...
        %             xy(cur_start,1)*xy(test_end,2) - xy(test_end,1)*xy(cur_start,2)];
        %         L = L/hypot(L(1),L(2));
        d = pts(:,1)*L(1) + pts(:,2)*L(2) + L(3);
        d2 = d.^2;
        if (all(d2<=max_dist^2))
            cur_end = test_end;
        else
            break;
        end
    end
    
    segs{end+1,1} = [cur_start,cur_end];
    cur_start = cur_end+1;
end

segs = cell2mat(segs);
diffs = segs(:,2)-segs(:,1);
segs = segs(diffs>min_pts,:);



if (false)
    figure(22);
    clf
    hold on;
    for i = 1:size(segs,1)
        myplot(xy(segs(i,1):segs(i,2),:),'.','color',rand(1,3));
    end
    hold off;
    axis equal;
    grid on;
    drawnow;
end




function L = fit_line(pts)

avg = mean(pts,1);
p = [pts(:,1)-avg(1),pts(:,2)-avg(2)];
[~,~,v] = svd(p,0);
L = [v(:,2); -pts(1,1)*v(1,2)-pts(1,2)*v(2,2)];
L = L/sqrt(L(1)^2+L(2)^2);
