function [P, frames] = distributePointsOnSphere(N)
  k = 1:N;
  h = -1 + 2*(k-1)/(N-1);
  theta = acos(h);
  phi = zeros(1,N);
  for i = 2:N-1
    phi(i) = mod(phi(i-1) + 3.6/(sqrt(N) * sqrt(1 - h(i)^2)), 2*pi);
  end
  x = cos(phi).*sin(theta);
  y = sin(phi).*sin(theta);
  z = cos(theta);
  P = [x; y; z];
  %   drawTreePoints(P, 'text', 'points')
  frames = zeros(N, 3, 3);
  for p = 1:N
    frame = zeros(3);
    frame(1:3,3) = -P(:,p);
    if abs(frame(1,3)) <= 1e-10 && abs(frame(2,3)) <= 1e-10
      frame(:,1) = [sign(frame(3,3)); 0; 0];
    else
      frame(2,1) = sqrt(frame(1,3)^2/(frame(2,3)^2 + frame(1,3)^2));
      frame(1,1) = -sign(frame(1,3)*frame(2,3))*sqrt(1-frame(2,1)^2);
    end
    frame(:,2) = cross(frame(:,3), frame(:,1));
    frames(p, :, :) = frame;
  end
  %   sphere()
  %   hold on
  %   plot3(P(1,:), P(2,:), P(3,:), 'r.')
end