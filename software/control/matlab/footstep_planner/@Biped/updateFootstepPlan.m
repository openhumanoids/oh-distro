function [X, exitflag] = updateFootstepPlan(biped, X, ndx_r, ndx_l, heightfun)

max_diag_dist = sqrt(biped.max_step_length^2 + biped.step_width^2);

X_orig = X;

x_flat = reshape(X([1,2,6],:), 1, []);

Xpos = X(1:6,:);
x_l = min(Xpos, [], 2);
x_u = max(Xpos, [], 2);
x_lb = x_l - abs(x_u - x_l) * 0.5;
x_ub = x_u + abs(x_u - x_l) * 0.5;
lb = repmat(x_lb, 1, length(X(1,:)));
ub = repmat(x_ub, 1, length(X(1,:)));
lb(6,:) = repmat(-pi, 1, length(X(1,:)));
ub(6,:) = repmat(pi, 1, length(X(1,:)));

lb(X(9:14,:)>0) = Xpos(X(9:14,:)>0);
ub(X(9:14,:)>0) = Xpos(X(9:14,:)>0);

ncon = 2 * (length(x_flat) - 3);
nvar = length(x_flat);
A = [eye(ncon/2, nvar) - [zeros(ncon/2, 3), eye(ncon/2, nvar-3)];
     -1 .* eye(ncon/2, nvar) + [zeros(ncon/2, 3), eye(ncon/2, nvar-3)]];
A = sparse(A);
b = repmat([biped.max_step_length / 2; biped.max_step_length / 2; biped.max_step_rot / 2], ncon / 3, 1);
         

[x_flat,fval,exitflag,output,lambda,grad] = fmincon(@cost, x_flat,A,b,[],[],...
  reshape(lb([1,2,6],:), 1, []), reshape(ub([1,2,6],:), 1, []),@nonlcon,...
  optimset('Algorithm', 'interior-point', 'MaxIter', 10, 'Display', 'off', 'TolX', 0.01));
% grad
% exitflag

X = X_orig;
X([1,2,6],:) = reshape(x_flat, 3, []);

function [c, ceq] = nonlcon(x_flat)
  X = X_orig;
  X([1,2,6],:) = reshape(x_flat, 3, []);
  [Xright, Xleft] = biped.stepLocations(X);
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

function c = cost(x_flat)
  X = X_orig;
  X([1,2,6],:) = reshape(x_flat, 3, []);
  c = biped.stepCost(X);
end

end