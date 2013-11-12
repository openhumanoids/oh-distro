function [x,dx] = headMarkerPos(params, use_fixed)
%NOTEST
if nargin < 2
  use_fixed = false;
end
  x = reshape(params,3,4);
  if (use_fixed)
    dx = zeros(12,0);
  else
    dx = eye(12);
  end
end