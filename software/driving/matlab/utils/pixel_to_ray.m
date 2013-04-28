function rays = pixel_to_ray(p,lens)

% Compute distorted rays
% Transform by inverse of pinhole matrix
invK = inv(lens.K);
rays_d = [p(:,1:2),ones(size(p,1),1)]*invK(1:2,:)';

% Determine distorted angle wrt view axis
ang_d = atan(hypot(rays_d(:,1),rays_d(:,2)));

% Determine undistorted angle wrt view axis
ang_u = lens.undist.func(lens.undist,ang_d);

% Rotate view axis by new angle in xz plane
rays = [sin(ang_u),zeros(size(ang_u)),cos(ang_u)];

% Rotate result in xy plane to align with original ray
thetas = atan2(rays_d(:,2),rays_d(:,1));
s = sin(thetas);
c = cos(thetas);
rays = [rays(:,1).*c - rays(:,2).*s, rays(:,1).*s + rays(:,2).*c, rays(:,3)];
