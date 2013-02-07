step_length = 0.3;
step_width = 0.5;
start_pos = [0;0;0;0;0;0];
max_rot = pi/16;
traj = turnGoTraj([start_pos, [2;2;0;0;0;0], [4;0;0;0;0;-pi/4]]);
[lambda, ndx_r, ndx_l] = constrainedFootsteps(traj, step_length, step_width, max_rot);
figure(21)
plotFootstepPlan(traj, lambda, ndx_r, ndx_l, step_width);

lambda_star = optimizeFootsteps(traj, lambda, step_length, step_width, max_rot);
figure(22)
plotFootstepPlan(traj, lambda_star, ndx_r, ndx_l, step_width);

