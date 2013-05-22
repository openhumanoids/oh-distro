function stats = error_stats(vars)

stats = struct();
stats.t = vars.t;
stats.vel.truth = vars.tvel;
stats.vel.est = vars.evel;
stats.vel.errors = stats.vel.truth - stats.vel.est;
stats.vel.rms_error = sqrt(mean(stats.vel.errors.^2,1));
stats.vel.max_error = max(abs(stats.vel.errors),[],1);
stats.vel.num_exceeding = sum(abs(stats.vel.errors)>2,1);
stats.vel.pct_exceeding = stats.vel.num_exceeding/numel(stats.t)*100;

stats.all.truth = [vars.tpos,vars.tvel,vars.tE,vars.trate];
stats.all.est = [vars.epos,vars.evel,vars.eE,vars.erate];
stats.all.errors = stats.all.truth - stats.all.est;
stats.all.stdev = std(stats.all.errors, [], 1);
segs = diff(vars.tpos,1,1);
total_distance_traveled = sum(sqrt(sum(segs.^2,2)));
stats.all.drift = stats.all.errors / total_distance_traveled;
stats.all.max_drift = max(stats.all.drift,[],1);