function [d, r] = stepDistance(X0, Xf)
  d = sqrt(sum((X0(1:2,:) - Xf(1:2,:)) .^2, 1));
  r = abs(Xf(6,:) - X0(6,:));
end

