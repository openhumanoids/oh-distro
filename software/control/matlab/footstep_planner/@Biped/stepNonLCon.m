function [c, ceq] = stepNonLCon(obj, X, ndx_r, ndx_l, heightfun)
  max_diag_dist = sqrt(obj.max_step_length^2 + obj.step_width^2);
  [Xright, Xleft] = obj.stepLocations(X);
  Xright(3,:) = heightfun(Xright(1:2,:));
  Xleft(3,:) = heightfun(Xleft(1:2,:));
  dist_alt_r = abs(Xright(:,1:end-1) - Xleft(:,2:end));
  dist_alt_l = abs(Xleft(:,1:end-1) - Xright(:,2:end));
  height_diff = [abs(Xright(3,ndx_r(1:end-1)) - Xright(3,ndx_r(2:end)))';
                 abs(Xleft(3,ndx_l(1:end-1)) - Xleft(3,ndx_l(2:end)))'];
  c = [reshape(dist_alt_r(1:2,:), [], 1) - max_diag_dist;
      reshape(dist_alt_l(1:2,:), [], 1) - max_diag_dist;
       height_diff - 0.5];
  ceq = 0;
end
