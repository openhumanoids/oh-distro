function predict (v,g,Q,WB,dt)
global XX PX

XX = [XX; v; g];
PX = blkdiag(PX, Q);

[XX,PX] = unscented_transform(@vehiclemod, @vehiclediff, XX, PX, WB,dt);

%

function x = vehiclemod(x, WB, dt)
V = x(end-1, :);
G = x(end, :);
x = x(1:end-2, :);

x(1:3, :) = vehicle_model(x, V,G, WB,dt);

%

function dx = vehiclediff(x1, x2)
dx = x1 - x2;
dx(3,:) = pi_to_pi(dx(3,:));
