function [d, r] = stepDistance(traj, lambda0, lambdaf, foot_angle, step_width)
  X0 = footstepLocations(traj, lambda0, foot_angle, step_width);
  Xf = footstepLocations(traj, lambdaf, foot_angle, step_width);
  d = sqrt(sum((X0(1:2,:) - Xf(1:2,:)) .^2, 1));
  r = abs(Xf(6,:) - X0(6,:));
end

