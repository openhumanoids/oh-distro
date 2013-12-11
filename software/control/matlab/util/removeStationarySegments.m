function [X_moving,T_moving] = removeStationarySegments(X,T,threshold)
  if nargin < 3, threshold = 1e-3; end
  nq = size(X,1)/2;
  % Smooth original data
  for i = 1:2*nq, X(i,:) = smooth(X(i,:)'); end
  % Find indices with norm(qdot) > threshold
  idx_moving = sum(X(nq+(1:nq),:).^2,1) > threshold;
  X_moving = X(:,idx_moving);
  T_moving = T(1:size(X_moving,2));
  % Smooth truncated data
  for i = 1:2*nq, X_moving(i,:) = smooth(X_moving(i,:)'); end
end

