function [d, r] = stepDistance(obj, X0, Xf, rescale)
  d = sqrt(sum((X0(1:2,:) - Xf(1:2,:)) .^2, 1));
  r = abs(Xf(6,:) - X0(6,:));
  if rescale
    theta = atan2(Xf(2,:) - X0(2,:), Xf(1,:) - X0(1,:)) - X0(6,:);
    scale = exp(mod(abs(theta), 2 * pi) + 1);
    % scale = mod(abs(theta - X0(6,:)), pi) ./ (pi/2) .* 10 + 1
    d = d .* scale;
  end
end

