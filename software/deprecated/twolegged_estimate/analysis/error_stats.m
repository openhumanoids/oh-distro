function stats = error_stats(vars)

% determine valid time range from first footstep to fall (if any)
min_idx = min(find(vars.left_con,1,'first'),find(vars.right_con,1,'first'));
min_idx = max(1,min_idx-20);
max_idx = find(any(vars.tE(:,1:2)>20*pi/180,2),1,'first');
max_idx = max(1,max_idx-100);
if (numel(max_idx) == 0)
    max_idx = numel(vars.left_con);
end

stats = struct();
stats.t = vars.t(min_idx:max_idx);
stats.vel.truth = vars.tvel(min_idx:max_idx,:);
stats.vel.est = vars.evel(min_idx:max_idx,:);
stats.vel.errors = stats.vel.truth - stats.vel.est;
stats.vel.rms_error = sqrt(mean(stats.vel.errors.^2,1));
stats.vel.max_error = max(abs(stats.vel.errors),[],1);
stats.vel.num_exceeding = sum(abs(stats.vel.errors)>0.2, 1);
stats.vel.pct_exceeding = stats.vel.num_exceeding/numel(stats.t)*100;

stats.all.truth = [vars.tpos(min_idx:max_idx,:),vars.tvel(min_idx:max_idx,:),...
    vars.tE(min_idx:max_idx,:),vars.trate(min_idx:max_idx,:)];
stats.all.est = [vars.epos(min_idx:max_idx,:),vars.evel(min_idx:max_idx,:),...
    vars.eE(min_idx:max_idx,:),vars.erate(min_idx:max_idx,:)];
stats.all.errors = stats.all.truth - stats.all.est;
stats.all.stdev = std(stats.all.errors, [], 1);
segs = diff(vars.tpos(min_idx:max_idx,:),1,1);
total_distance_traveled = sum(sqrt(sum(segs.^2,2)));
stats.all.drift = stats.all.errors / total_distance_traveled;
stats.all.max_drift = max(stats.all.drift,[],1);
