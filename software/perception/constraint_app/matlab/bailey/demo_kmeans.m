function demo_kmeans()
%function demo_kmeans()
%
% Demonstrate the k-means clustering algorithm.
%
% Tim Bailey 2005.

s1 = gauss_samples([0;0], [1,0;0,2], 1000);
s2 = gauss_samples([3;4], [3,0.5;0.5,2], 2000);
s = [s1 s2];

xi = [0 0; -1 1];
[x,P,w] = kmeans(s, xi, 10);

figure
plot(s(1,:), s(2,:), '.', xi(1,:), xi(2,:), 'o', x(1,:), x(2,:), 'pr')
legend('samples','initial centres','final centres')

hold on
for i=1:size(x,2)
    e = sigma_ellipse(x(:,i), P(:,:,i), 2);
    plot(e(1,:), e(2,:), 'r')
end
