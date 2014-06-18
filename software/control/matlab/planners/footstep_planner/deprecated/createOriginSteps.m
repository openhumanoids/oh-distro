function X = createOriginSteps(biped, foot_orig, right_foot_lead)
%NOTEST
% Create the step entries corresponding to the initial poses of the feet, respecting the desired step order
if right_foot_lead
  X(1) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 1), 'is_right_foot', true);
  X(2) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 0), 'is_right_foot', false);
else
  X(1) = struct('pos', biped.footOrig2Contact(foot_orig.left, 'center', 1), 'is_right_foot', false);
  X(2) = struct('pos', biped.footOrig2Contact(foot_orig.right, 'center', 0), 'is_right_foot', true);
end

