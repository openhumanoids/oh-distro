function terrain_pts = sampleSwingTerrain(biped, last_pos, next_pos, contact_width, options)

if nargin < 5; options = struct(); end
if ~isfield(options, 'nrho'); options.nrho = 10; end

step_dist_xy = sqrt(sum((next_pos(1:2) - last_pos(1:2)).^2));
if ~isfield(options, 'nlambda'); options.nlambda = max([ceil(step_dist_xy / 0.02),3]); end

lambda_hat = (next_pos(1:2) - last_pos(1:2)) / step_dist_xy;

rho_hat = [0, -1; 1, 0] * lambda_hat;

terrain_pts = zeros(2, options.nlambda);
lambdas = linspace(0, step_dist_xy, options.nlambda);
rhos = linspace(-contact_width, contact_width, options.nrho);
[R, L] = meshgrid(rhos, lambdas);
xy = bsxfun(@plus, last_pos(1:2), bsxfun(@times, reshape(R, 1, []), rho_hat) + bsxfun(@times, reshape(L, 1, []), lambda_hat));
z = reshape(biped.getTerrainHeight(xy), size(R));
% z = medfilt2(z, 'symmetric');
terrain_pts(2, :) = max(z, [], 2);
terrain_pts(1,:) = lambdas;

terrain_pts(2,1) = last_pos(3);
terrain_pts(2,end) = next_pos(3);

if step_dist_xy > 0.01
  terrain_pts(:,end+1) = terrain_pts(:,end) + [0;-1]; % stupid hack to ensure non-colinearity
  terrain_pts = terrain_pts(:, convhull(terrain_pts(1,:), terrain_pts(2,:), 'simplify', true));
  terrain_pts = terrain_pts(:,end:-1:1); % convert counterclockwise to clockwise convex hull
  terrain_pts = terrain_pts(:, 1:find(terrain_pts(1,:) >= step_dist_xy, 1, 'first'));
  terrain_pts(2,1) = last_pos(3);
  terrain_pts(2,end) = next_pos(3);
end

xy = bsxfun(@plus, last_pos(1:2), bsxfun(@times,terrain_pts(1,:), lambda_hat));
plot_lcm_points([xy; terrain_pts(2,:)]', repmat([1 1 0], size(xy, 2), 1), 103, 'Terrain convex hull', 2, 1);

end
