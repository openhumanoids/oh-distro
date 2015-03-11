function t_warped = warpTime(t, acceleration_param)
  % @param acceleration_param - Scalar parameter greater than or equal to 2 that
  %                             adjusts the acceleration profile. Higher values
  %                             yield more gradual accelerations.
  alpha = 1/acceleration_param;
  C = (alpha*(1/2)^(alpha - 1))^(-1);

  first_half_idx = (t <= 0.5);
  t_warped = zeros(size(t));
  t_warped(first_half_idx) = C*t(first_half_idx).^alpha;
  t_warped(~first_half_idx) = 2*C*(1/2)^alpha - C*(1-t(~first_half_idx)).^alpha;
end
