function [X, exitflag] = updateFastFootsteps(biped, X, fixed_steps, ndx_r, ndx_l, heightfun)

max_diag_dist = sqrt(biped.max_step_length^2 + biped.step_width^2);

x_flat = reshape(X([1,2,6],:), 1, []);
X = locate_step_centers(x_flat);
x_flat = reshape(X([1,2,6],:), 1, []);

x_l = min(X, [], 2);
x_u = max(X, [], 2);
x_lb = x_l - abs(x_u - x_l) * 0.5;
x_ub = x_u + abs(x_u - x_l) * 0.5;
lb = repmat(x_lb, 1, length(X(1,:)));
ub = repmat(x_ub, 1, length(X(1,:)));
lb(6,:) = repmat(-pi, 1, length(X(1,:)));
ub(6,:) = repmat(pi, 1, length(X(1,:)));

ndx_fixed = find(any(cellfun(@(x) ~isempty(x),fixed_steps),2));
lb(:, ndx_fixed) = X(:, ndx_fixed);
ub(:, ndx_fixed) = X(:, ndx_fixed);

ncon = 2 * (length(x_flat) - 3);
nvar = length(x_flat);
A = [eye(ncon/2, nvar) - [zeros(ncon/2, 3), eye(ncon/2, nvar-3)];
     -1 .* eye(ncon/2, nvar) + [zeros(ncon/2, 3), eye(ncon/2, nvar-3)]];
A = sparse(A);
b = repmat([biped.max_step_length / 2; biped.max_step_length / 2; biped.max_step_rot / 2], ncon / 3, 1);
         

[x_flat,fval,exitflag,output,lambda,grad] = fmincon(@cost, x_flat,A,b,[],[],...
  reshape(lb([1,2,6],:), 1, []), reshape(ub([1,2,6],:), 1, []),@nonlcon,...
  optimset('Algorithm', 'interior-point', 'MaxIter', 10, 'Display', 'off', 'TolX', 0.01));
grad
exitflag

X = locate_step_centers(x_flat);

function [c, ceq] = nonlcon(x_flat)
  X = locate_step_centers(x_flat);
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
  X = locate_step_centers(x_flat);
  [d, r] = biped.stepDistance(X(:,1:(end-1)), X(:,2:end),1);
  c = sum(d.^2 + (r .* (biped.max_step_length / biped.max_step_rot)).^2 + (d .* r .* (10 * biped.max_step_length / biped.max_step_rot)).^2);

%   
%   X = locate_step_centers(x_flat);
%   deviation = atan2(X(2,2:end) - X(2,1:end-1), X(1,2:end) - X(1,1:end-1)) - X(6,1:end-1);
%   c = sum(deviation .^ 2);
end

function X = locate_step_centers(x_flat)
  x_flat = reshape(x_flat, 3, []);
  X = [[x_flat(1:2,:)]; zeros(3, length(x_flat(1,:))); [x_flat(3,:)]];
  for i = 1:length(X(1,:))
    if ~isempty(fixed_steps{i,1}) && ~isempty(fixed_steps{i,2})
      X(:,i) = mean([fixed_steps{i,1}, fixed_steps{i,2}],2);
    elseif ~isempty(fixed_steps{i,1})
      yaw = fixed_steps{i,1}(6);
      angle = yaw - biped.foot_angles(1);
      X(:,i) = fixed_steps{i,1} + (biped.step_width / 2) * [cos(angle); sin(angle);0;0;0;0];
    elseif ~isempty(fixed_steps{i,2})
      yaw = fixed_steps{i,2}(6);
      angle = yaw - biped.foot_angles(2);
      X(:,i) = fixed_steps{i,2} + (biped.step_width / 2) * [cos(angle); sin(angle);0;0;0;0];
    end
  end
end
end