function segs = split_scan_into_runs(scan, range_range, theta_range, min_pts, max_diff)

thetas = scan.theta_min + scan.theta_step*(0:numel(scan.ranges)-1);
thetas = thetas(:);
ranges = scan.ranges;
bad_ind = ranges<range_range(1) | ranges>range_range(2);
bad_ind = bad_ind | thetas<theta_range(1) | thetas>theta_range(2);

ranges(bad_ind) = 0;
d = abs(diff(ranges));
starts = [1;find(d>max_diff)+1];
ends = [starts(2:end)-1;numel(d)+1];
good_segs = ~bad_ind(starts) & ~bad_ind(ends) & (ends-starts+1)>=min_pts;
segs = [starts(good_segs),ends(good_segs)];
