function drawFootTrajectories(link_constraints)

poses = link_constraints.poses;
dposes = link_constraints.dposes;
ts = link_constraints.ts;
a0 = link_constraints.a0;
a1 = link_constraints.a1;
a2 = link_constraints.a2;
a3 = link_constraints.a3;

n = 50;

X = zeros(size(poses,1), (size(poses,2)-1)*n);
V = zeros(size(X));
T = zeros(1, size(X,2));
for j = 1:size(poses,2)-1
    t0 = ts(j);
    a0i = repmat(a0(:,j), 1, n);
    a1i = repmat(a1(:,j), 1, n);
    a2i = repmat(a2(:,j), 1, n);
    a3i = repmat(a3(:,j), 1, n);
    ti = linspace(0, ts(j+1)-t0, n);
    ti = repmat(ti, size(poses,1), 1);
    X(:,(j-1)*n+(1:n)) = a0i + a1i .* ti + a2i .* ti.^2 + a3i .* ti.^3;
    V(:,(j-1)*n+(1:n)) = a1i + 2*a2i .* ti + 3*a3i .* ti.^2;
    T((j-1)*n+(1:n)) = ti(1,:) + t0;
end

figure()
clf
hold on
plot(T, X(1,:), T, X(2,:), T, X(3,:));
quiver(T, X(3,:), zeros(size(T)), V(3,:));
legend('x1', 'x2', 'x3')