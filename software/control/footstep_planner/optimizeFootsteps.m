function [lambda_star, Xright, Xleft] = optimizeFootsteps(traj, lambda, step_length, step_width, max_rot, ndx_r, ndx_l, fixed_steps)

foot_angles = [-pi/2, pi/2];

if nargin < 8
  fixed_steps = repmat({[]}, length(lambda), 2);
end

  function c = cost(lambda)
    c = 0;
    lambda(1) = 0;
    lambda(end) = 1;
    for current_foot = 1:2
      X0 = footstepLocations(traj, lambda(1:(end-1)), foot_angles(current_foot), step_width);
      Xf = footstepLocations(traj, lambda(2:end), foot_angles(current_foot), step_width);
      for j = 1:(length(lambda))
        if ~isempty(fixed_steps{j,current_foot})
          if j < length(lambda)
            X0(1:2,j) = fixed_steps{j,current_foot};
          end
          if j > 1
            Xf(1:2,j-1) = fixed_steps{j,current_foot};
          end
        end
      end
      [d, r] = stepDistance(X0, Xf);
      dist = d + r .* step_length / max_rot;
      c = c + sum(dist .^ 2);
    end
  end

lambda_star = fmincon(@cost, lambda, [], [], [], [], zeros(size(lambda)), ones(size(lambda)),[],...
  optimset('MaxTime', 10, 'Display', 'off'));
Xright = footstepLocations(traj, lambda_star(ndx_r), -pi/2, step_width);
Xleft = footstepLocations(traj, lambda_star(ndx_l), pi/2, step_width);
for j = 1:length(ndx_r)
  if ~isempty(fixed_steps{ndx_r(j), 1})
    Xright(1:2,j) = fixed_steps{ndx_r(j), 1};
  end
end
for j = 1:length(ndx_l)
  if ~isempty(fixed_steps{ndx_l(j), 1})
    Xright(1:2,j) = fixed_steps{ndx_l(j), 1};
  end
end

end