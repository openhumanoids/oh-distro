function plotFootstepPlan(X, Xright, Xleft)

% X = traj.eval(linspace(0,1));

plot(Xright(1,:), Xright(2,:), 'go',...
  Xleft(1,:), Xleft(2,:), 'ro')


hold on
if ~isempty(X)
  plot(X(1,:), X(2,:), 'b')
end
axis equal
quiver(Xright(1,:), Xright(2,:), cos(Xright(6,:)), sin(Xright(6,:)),'Color', 'g', 'AutoScaleFactor', 0.2);
quiver(Xleft(1,:), Xleft(2,:), cos(Xleft(6,:)), sin(Xleft(6,:)),'Color', 'r', 'AutoScaleFactor', 0.2);
hold off

end