function [y,ydot,yddot] = evalSplineSegment(t,a0,a1,a2,a3)
  y = a0 + a1*t + a2*t^2 + a3*t^3;
  if nargout > 1
    ydot = a1 + 2*a2*t + 3*a3*t^2;
  end
  if nargout > 2
    yddot = 2*a2 + 6*a3*t;
  end
end

