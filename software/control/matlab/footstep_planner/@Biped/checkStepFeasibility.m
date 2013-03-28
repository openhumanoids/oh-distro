function c = checkStepFeasibility(biped, p0, pf, p0_is_right_foot)

  sizecheck(p0(1,:), size(p0_is_right_foot));

  if size(p0, 1) == 3
    p0 = [p0(1, :); p0(2, :); zeros(3, size(p0, 2)); p0(3, :)];
  end

  max_step_width = 0.35;
  min_step_width = 0.20;

  % theta=0 is the direction the stance foot is pointing
  theta_max = 2*pi/3;
  theta_min = pi/4;
  theta_mean = mean([theta_min, theta_max]);

  c = zeros(3, size(p0, 2));
  for j = 1:size(p0, 2)
    theta = atan2(pf(2, j) - p0(2, j), pf(1, j) - p0(1, j)) - p0(6, j);
    r = sqrt(sum((pf(1:2,j) - p0(1:2,j)).^2));
    if ~p0_is_right_foot(j)
      theta = -theta;
    end

    d = mod(theta - theta_mean, 2*pi);
    if d > pi
      d = 2*pi - d;
    end
    c(1, j) = d - (theta_max - theta_min) / 2;
    c(2, j) = max(r - max_step_width, min_step_width - r);

    phi = pf(6, j) - p0(6, j);
    c(3, j) = abs(phi) - biped.max_step_rot;
  end
  c = reshape(c, [], 1);
end


