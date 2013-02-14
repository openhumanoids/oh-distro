function plotFootstepPlan(traj, Xright, Xleft)
X = traj.eval(linspace(0,1));

plot(Xright(1,:), Xright(2,:), 'go',...
  Xleft(1,:), Xleft(2,:), 'ro', ...
  X(1,:), X(2,:), 'b')
axis equal

end