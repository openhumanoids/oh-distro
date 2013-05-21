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
