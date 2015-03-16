function drawFootTrajectories(link_constraints)

ts = link_constraints.ts;
coefs = link_constraints.coefs;

n = 50;

tt = linspace(ts(1), ts(end), n * length(ts));
pp = mkpp(ts, coefs, size(coefs, 1));
X = ppval(pp, tt);
V = ppval(fnder(pp), tt);

figure()
clf
hold on
plot(tt, X(1,:), tt, X(2,:), tt, X(3,:));
quiver(tt, X(3,:), zeros(size(tt)), V(3,:));
legend('x1', 'x2', 'x3')