function c = checkStepFeasibility(biped, p0, pf, p0_is_right_foot, max_forward_step)

  if nargin < 5
    max_forward_step = biped.max_forward_step;
  end

  sizecheck(p0(1,:), size(p0_is_right_foot));

  if size(p0, 1) == 3
    p0 = [p0(1, :); p0(2, :); zeros(3, size(p0, 2)); p0(3, :)];
  end

  x_max = max_forward_step;
  x_min = -biped.max_backward_step;

  c = zeros(4, size(p0, 2));
  for j = 1:size(p0, 2)
    u = rotz(-p0(6, j)) * (pf(1:3, j) - p0(1:3, j));
    if ~p0_is_right_foot(j)
      u(2) = -u(2);
    end
    if u(1) >= 0
      c(1, j) = u(1) - x_max;
      x_feas_frac = max(0, 1 - u(1) / x_max);
    else
      c(1, j) = x_min - u(1);
      x_feas_frac = max(0, 1 - u(1) / x_min);
    end
    % c(1, j) = abs(u(1) - x_mean) - (x_max - x_min) / 2;
    % x_feas_frac = max(0, 1 - abs(u(1) - x_mean) / ((x_max - x_min) / 2));
    if u(2) >= biped.nom_step_width
      % c(2, j) = u(2) - interp1([0, 1], [biped.nom_step_width, biped.max_step_width], x_feas_frac);
      c(2, j) = u(2) - biped.max_step_width;
    else
      c(2, j) = biped.min_step_width - u(2);
      % c(2, j) = interp1([0, 1], [biped.nom_step_width, biped.min_step_width], x_feas_frac) - u(2);
      if c(2, j) > 0
        c(2, j) = c(2, j) * 10;
      end
    end
    phi = diff(unwrap([pf(6, j), p0(6,j)]));
    % phi = angleDiff(pf(6, j), p0(6, j));
    % phi = pf(6, j) - p0(6, j);
    c(3, j) = abs(phi) - biped.max_step_rot;
    % c(3, j) = abs(phi) - interp1([0, 1], [0, biped.max_step_rot], x_feas_frac);

    c(4, j) = abs(pf(3, j) - p0(3, j)) - biped.max_step_dz;
  end
  c = reshape(c, [], 1);
end


