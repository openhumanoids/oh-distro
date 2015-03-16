function t_warped = warpTime(t, acceleration_param, t_acc, t_dec)
  % @param acceleration_param - Scalar parameter greater than or equal to 2 that
  %                             adjusts the acceleration profile. Higher values
  %                             yield more gradual accelerations.
  % @param t_acc - Scalar in (0,1) indicating how much of the
  %                            trajectory's duration is spent accelerating.
  %                            The sum of t_acc and
  %                            t_dec must be less than or equal to
  %                            1.
  %                            @default 0.5
  % @param t_dec - Scalar in (0,1) indicating how much of the
  %                            trajectory's duration is spent decelerating
  %                            @default equal to t_acc
  if nargin < 2, acceleration_param = 2; end
  if nargin < 3, t_acc = 0.5; end
  if nargin < 4, t_dec = t_acc; end
  assert(t_acc + t_dec <= 1, 't_acc + t_dec must be less than or equal to 1');
  t_max = 1 - t_acc - t_dec;

  % Set up scaling constants
  alpha = 1/acceleration_param;
  C_acc = (alpha*(t_acc)^(alpha - 1))^(-1);
  C_dec = (alpha*(t_dec)^(alpha - 1))^(-1);

  % Compute indices for acceleration, max velocity, and deceleration segments
  idx_acc = (t <= t_acc);
  idx_dec = (t >= (1-t_dec));
  idx_max = (~idx_acc & ~idx_dec);

  % Generate warped time
  t_warped = zeros(size(t));
  t_warped(idx_acc) = C_acc*t(idx_acc).^alpha;
  t_warped(idx_max) = C_acc*(t_acc)^alpha - t_acc + t(idx_max);
  t_warped(idx_dec) = C_acc*(t_acc)^alpha + C_dec*(t_dec)^alpha + t_max - C_dec*(1-t(idx_dec)).^alpha;
end
