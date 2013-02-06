step_length = 0.3;
step_width = 0.5;
start_pos = [0;0;0;0;0;0];
goal_pos = [2;2;0;0;0;0];
% traj = cubicSplineTraj(start_pos, goal_pos);
traj = turnGoTraj(start_pos, goal_pos);
% [Xright, Xleft, X] = equalFootsteps(traj, step_length, step_width);
[Xright, Xleft, X] = constrainedFootsteps(traj, step_length, step_width);
figure(21)
plot(Xright(1,:), Xright(2,:), 'go',...
  Xleft(1,:), Xleft(2,:), 'ro', ...
  X(1,:), X(2,:), 'b')
axis equal
drawnow