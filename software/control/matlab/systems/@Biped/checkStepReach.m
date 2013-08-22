function c = checkStepReach(biped, p0, pf, p0_is_right_foot, options)
  % For a single initial foot pose in p0 and a list of potential poses for the other foot in pf, return a matrix c with one column for each pose in pf. If all(c(:,j)) < 0, then the step pf(:,j) is reachable from p0.

  if nargin < 5
    options = struct();
  end
  [A, b] = getFootstepLinearCons(biped, options);
  u = Biped.relativeSteps(p0, pf, p0_is_right_foot);
  c = bsxfun(@minus, A * u, b);
end


