function x = encodeCollocationSteps(steps)
nsteps = size(steps, 2);
x = zeros(12,nsteps);
x(1:6,:) = steps;
x(7:12,1) = steps(:,1);
for j = 2:nsteps
  R = rotmat(-steps(6,j-1));
  x(7:12,j) = [R * (steps(1:2,j) - steps(1:2,j-1));
              steps(3:6,j) - steps(3:6,j-1)];
end
x = reshape(x, [], 1);
end
