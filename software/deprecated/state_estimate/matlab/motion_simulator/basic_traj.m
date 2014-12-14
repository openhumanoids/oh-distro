function val = basic_traj(iterations, knobs, Hd)

alpha = knobs.alpha;
beta = knobs.beta;
eta = knobs.eta;


% horizontal position
temp = cumsum(randn(iterations,1)*eta);

% inner detrend represents steps
vel_csum(:,1) = filter(Hd,temp - (beta)*detrend(temp,'linear',1:knobs.step:iterations));
vel_csum(:,1) = vel_csum(:,1) - vel_csum(1,1);
vel_csum(:,1) = vel_csum(:,1) - alpha * (1:iterations)'./iterations .* vel_csum(end,1);
vel_csum(:,1) = vel_csum(:,1) - vel_csum(1,1);

val = vel_csum;