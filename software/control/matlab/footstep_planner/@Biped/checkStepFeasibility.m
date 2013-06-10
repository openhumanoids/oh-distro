function c = checkStepFeasibility(biped, p0, pf, p0_is_right_foot, options)


  if nargin < 5
    options = struct();
  end

  if ~isfield(options, 'forward_step')
    options.forward_step = biped.max_forward_step;
  end
  if ~isfield(options, 'nom_step_width')
    options.nom_step_width = biped.nom_step_width;
  end
  if ~isfield(options, 'max_step_width')
    options.max_step_width = max([biped.max_step_width, options.nom_step_width + 0.01]);
  end
  if ~isfield(options, 'min_step_width')
    options.min_step_width = min([biped.min_step_width, options.nom_step_width - 0.01]);
  end
  if ~isfield(options, 'backward_step')
    options.backward_step = biped.max_backward_step;
  end
  if ~isfield(options, 'max_step_rot')
    options.max_step_rot = biped.max_step_rot;
  end
  if ~isfield(options, 'max_step_dz')
    options.max_step_dz = biped.max_step_dz;
  end

  sizecheck(p0(1,:), size(p0_is_right_foot));

  if size(p0, 1) == 3
    p0 = [p0(1, :); p0(2, :); zeros(3, size(p0, 2)); p0(3, :)];
  end

  x_max = options.forward_step;
  x_min = -options.backward_step;

  c = zeros(4, size(p0, 2));
  for j = 1:size(p0, 2)
    u = rotz(-p0(6, j)) * (pf(1:3, j) - p0(1:3, j));
    if ~p0_is_right_foot(j)
      u(2) = -u(2);
    end
    if u(1) >= 0
      c(1, j) = u(1) - x_max;
%       x_feas_frac = max(0, 1 - u(1) / x_max);
    else
      c(1, j) = x_min - u(1);
%       x_feas_frac = max(0, 1 - u(1) / x_min);
    end
    if u(2) >= options.nom_step_width
      c(2, j) = u(2) - options.max_step_width;
    else
      c(2, j) = options.min_step_width - u(2);
      if c(2, j) > 0
        c(2, j) = c(2, j) * 10;
      end
    end
    phi = angleDiff(pf(6, j), p0(6, j));
    c(3, j) = abs(phi) - options.max_step_rot;

    c(4, j) = abs(pf(3, j) - p0(3, j)) - options.max_step_dz;
  end
  c = reshape(c, [], 1);
end


