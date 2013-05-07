function terrain_pts = terrainSample(biped, last_pos, next_pos, contact_radius, nlambda, nrho)

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));
lambda_hat = (next_pos(1:2) - last_pos(1:2)) / step_dist_xy;

rho_hat = [0, -1; 1, 0] * lambda_hat;

terrain_pts = zeros(2, nlambda);
lambdas = linspace(0, step_dist_xy, nlambda);
rhos = linspace(-contact_radius, contact_radius, nrho);
% figure(1)
% clf
% hold on
for j = 1:nlambda
  terrain_pts(1,j) = lambdas(j);
  xy = bsxfun(@plus, bsxfun(@times, rhos, rho_hat),lambdas(j) * lambda_hat + last_pos(1:2));
%   plot(xy(1,:), xy(2,:), 'bo')
  terrain_pts(2,j) = max(biped.getTerrainHeight(xy));
end
