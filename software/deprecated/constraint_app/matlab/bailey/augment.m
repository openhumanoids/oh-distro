function augment(z,R)

% add new features to state
for i=1:size(z,2)
    add_one_z(z(:,i),R);
end

%
%

function add_one_z(z,R)
global XX PX

XX = [XX; z];
PX = blkdiag(PX, R);
[XX,PX] = unscented_transform(@augment_model, [], XX,PX);

%
%

function x = augment_model(x);
phi = x(3, :);
r = x(end-1, :);
b = x(end, :);
s = sin(phi + b); 
c = cos(phi + b);

x(end-1, :) = x(1,:) + r.*c;
x(end, :)   = x(2,:) + r.*s;
