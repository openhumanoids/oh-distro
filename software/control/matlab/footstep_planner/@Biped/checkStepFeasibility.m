function c = checkStepFeasibility(biped, p0, pf, p0_is_right_foot)

  sizecheck(p0(1,:), size(p0_is_right_foot));

  if size(p0, 1) == 3
    p0 = [p0(1, :); p0(2, :); zeros(3, size(p0, 2)); p0(3, :)];
  end


  max_forward_step = 0.25;
  max_backward_step = 0.15;
  max_step_width = 0.35;
  min_step_width = 0.18;
  nom_step_width = 0.2;

  x_max = max_forward_step;
  x_min = -max_backward_step;
  x_mean = mean([x_max, x_min]);

  c = zeros(3, size(p0, 2));
  for j = 1:size(p0, 2)
    u = rotz(-p0(6, j)) * (pf(1:3, j) - p0(1:3, j));
    if ~p0_is_right_foot(j)
      u(2) = -u(2);
    end
    c(1, j) = abs(u(1) - x_mean) - (x_max - x_min) / 2;
    x_feas_frac = max(0, 1 - abs(u(1) - x_mean) / ((x_max - x_min) / 2));
    if u(2) >= nom_step_width
%       c(2, j) = u(2) - interp1([0, 1], [nom_step_width, max_step_width], x_feas_frac);
      c(2, j) = u(2) - max_step_width;
    else
      c(2, j) = min_step_width - u(2);
%       c(2, j) = interp1([0, 1], [nom_step_width, min_step_width], x_feas_frac) - u(2);
    end
    phi = pf(6, j) - p0(6, j);
%     c(3, j) = abs(phi) - biped.max_step_rot;
    c(3, j) = abs(phi) - interp1([0, 1], [0, biped.max_step_rot], x_feas_frac);
  end
  c = reshape(c, [], 1);
end


