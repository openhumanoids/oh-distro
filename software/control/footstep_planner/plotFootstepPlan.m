function plotFootstepPlan(traj, lambda, ndx_r, ndx_l, step_width)

Xright = footstepLocations(traj, lambda(ndx_r), -pi/2, step_width);
Xleft = footstepLocations(traj, lambda(ndx_l), pi/2, step_width);
X = traj.eval(linspace(0,1));

plot(Xright(1,:), Xright(2,:), 'go',...
  Xleft(1,:), Xleft(2,:), 'ro', ...
  X(1,:), X(2,:), 'b')
axis equal

end