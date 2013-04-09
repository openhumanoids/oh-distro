
function draw_unitsphere(fid)

% Generate the x, y, and z data for the sphere
r = 1 * ones(50, 50); % radius is 1
[th, phi] = meshgrid(linspace(0, 2*pi, 50), linspace(-pi, pi, 50));
[x,y,z] = sph2cart(th, phi, r);
x = x + 0;  % center at 16 in x-direction
y = y + 0;  % center at 40 in y-direction
z = z + 0;   % center at 2 in z-direction
% Let's say that this is how we make the existing 3D plot


figure(fid)
hold on;
surface(x,y,z,'FaceColor',[1 1 1],'FaceALpha',1,'EdgeAlpha',0.05)
%surface(x,y,z,'FaceColor', [0.9 0.9 0.9],'FaceALpha',0.0,'EdgeAlpha',0.05)
axis equal;
hold off;
end