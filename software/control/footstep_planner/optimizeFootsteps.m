function [lambda_star] = optimizeFootsteps(traj, lambda, step_length, step_width, max_rot)

disp('running footstep placement optimization');
foot_angles = [-pi/2, pi/2];

  function c = cost(lambda)
    c = 0;
    lambda(1) = 0;
    lambda(end) = 1;
    for current_foot = 1:2
      [d, r] = stepDistance(traj, lambda(1:(end-1)), lambda(2:end), foot_angles(current_foot), step_width);
      dist = d + r .* step_length / max_rot;
      c = c + sum(dist .^ 2);
    end
  end

lambda_star = fmincon(@cost, lambda, [], [], [], [], zeros(size(lambda)), ones(size(lambda)),[],...
  optimset('MaxTime', 10));

disp('footstep placement optimization complete');

end