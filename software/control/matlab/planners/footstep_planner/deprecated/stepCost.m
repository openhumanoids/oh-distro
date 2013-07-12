function c = stepCost(obj, X)
  [d, r] = obj.stepDistance(X(:,1:(end-1)), X(:,2:end),1);
  % c = sum(d.^2 + (r .* (obj.max_step_length / obj.max_step_rot)).^2 + (d .* r .* (10 * obj.max_step_length / obj.max_step_rot)).^2);
  c = sum(d.^2 + (r .* (obj.max_step_length / obj.max_step_rot)).^2);
end