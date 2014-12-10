function pix = ray_to_pixel(rays,lens)

% Compute undistorted angle of ray wrt view axis
ang_u = atan2(hypot(rays(:,1),rays(:,2)),rays(:,3));

% Determine distorted angle wrt view axis
ang_d = lens.dist.func(lens.dist,ang_u);

% Rotate view axis by new angle in xz plane
pts = [sin(ang_d),zeros(size(ang_d)),cos(ang_d)];

% Rotate result in xy plane to align with original ray
thetas = atan2(rays(:,2),rays(:,1));
s = sin(thetas);
c = cos(thetas);
pts = [pts(:,1).*c - pts(:,2).*s, pts(:,1).*s + pts(:,2).*c, pts(:,3)];

% Transform by pinhole matrix
pts = pts*lens.K';

% Projective divide
pix = [pts(:,1:2)./pts(:,[3,3]),pts(:,3)];
