function [X, exitflag] = updateFootstepPlan(biped, X, heightfun)

%% Adjust the number of footsteps, if necessary
% [Xright, Xleft] = biped.stepLocations(X);
Xright = biped.stepCenter2FootCenter(X, 1);
Xleft = biped.stepCenter2FootCenter(X, 0);

ndx_fixed = find(any(X(9:14,:)));

[d_r, r_r] = biped.stepDistance(Xright(1:6,1:(end-1)), Xright(1:6,2:end), 0);
[d_l, r_l] = biped.stepDistance(Xleft(1:6,1:(end-1)), Xleft(1:6,2:end), 0);
for n = 1:(length(ndx_fixed)-1)
  num_steps = ndx_fixed(n+1) - ndx_fixed(n);
  dist = max(sum(d_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
             sum(d_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
  rot = max(sum(r_r(ndx_fixed(n):(ndx_fixed(n+1)-1))),...
            sum(r_l(ndx_fixed(n):(ndx_fixed(n+1)-1))));
  if  ((dist > num_steps * biped.max_step_length * .4 ...
        || rot > num_steps * biped.max_step_rot * .4) ...
      && (num_steps > 1))
    j = ndx_fixed(n);
    X(:,j+3:end+2) = X(:,j+1:end);
    X(:,[j+1,j+2]) = interp1([0,1], X(:,[j,j+1])', [1/3, 2/3])';
    X(8,:) = int32(X(8,:));
    X(7, :) = (0:length(X(1,:))-1) * biped.step_time / 2;
    X(9:14,[j+1,j+2]) = 0;
    break
  elseif (dist < num_steps * biped.max_step_length * 0.15 ...
          && rot < num_steps * biped.max_step_rot * 0.15) ...
      && (num_steps > 2)
    j = ndx_fixed(n);
    X(:,j+1:end-2) = X(:,j+3:end);
    X(:,end-1:end) = [];
    X(7, :) = (0:length(X(1,:))-1) * biped.step_time / 2;
    break
  end
end
total_steps = length(X(1,:));
ndx = biped.getStepNdx(total_steps);

%% Set up and run the optimization problem
  
max_diag_dist_sq = (biped.max_step_length^2 + biped.step_width^2);
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
  % [Xright, Xleft] = biped.stepLocations(X);
  Xright = biped.stepCenter2FootCenter(X, 1);
  Xleft = biped.stepCenter2FootCenter(X, 0);
  Xright(3,:) = heightfun(Xright(1:2,:));
  Xleft(3,:) = heightfun(Xleft(1:2,:));


  dist_alt_r = sum((Xright(1:2,1:end-1) - Xleft(1:2,2:end)).^2, 1);
  dist_alt_l = sum((Xleft(1:2,1:end-1) - Xright(1:2,2:end)).^2, 1);
  % dist_alt_r = abs(Xright(:,1:end-1) - Xleft(:,2:end));
  % dist_alt_l = abs(Xleft(:,1:end-1) - Xright(:,2:end));
  

  height_diff = [abs(Xright(3,ndx.right(1:end-1)) - Xright(3,ndx.right(2:end)))';
                 abs(Xleft(3,ndx.left(1:end-1)) - Xleft(3,ndx.left(2:end)))'];
  % c = [reshape(dist_alt_r(1:2,:), [], 1) - max_diag_dist;
  %     reshape(dist_alt_l(1:2,:), [], 1) - max_diag_dist;
  %      height_diff - 0.5];
  c = [dist_alt_r' - max_diag_dist_sq;
       dist_alt_l' - max_diag_dist_sq;
       -(dist_alt_r' - biped.min_foot_proximity^2);
       -(dist_alt_l' - biped.min_foot_proximity^2);
       height_diff - 0.5];
  ceq = 0;
end

function c = cost(x_flat)
  X = X_orig;
  X([1,2,6],:) = reshape(x_flat, 3, []);
  c = biped.stepCost(X);
end

end