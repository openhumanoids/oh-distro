function [X, exitflag] = updateRLFootstepPlan(biped, X, foot_goals, options, heightfun)

  max_diag_dist = sqrt(biped.max_step_width^2 + biped.forward_step^2);

% if options.max_num_steps - (length(X)-2) >= 1
%   if X(end-1).is_right_foot
%     goal = foot_goals.right;
%   else
%     goal = foot_goals.left;
%   end
%   c = biped.checkStepFeasibility(X(end-2).pos, goal, X(end-2).is_right_foot);
%   if any(c > 0)
%     if X(end-3).is_right_foot
%       goal = foot_goals.right;
%     else
%       goal = foot_goals.left;
%     end
%     new_X.pos = interp1([0, 1], [X(end-3).pos, goal]', 0.5)';
%     new_X.time = 0;
%     new_X.id = biped.getNextStepID();
%     new_X.pos_fixed = zeros(6, 1);
%     new_X.is_right_foot = X(end-3).is_right_foot;
%     X = [X(1:end-2), new_X, X(end), X(end-1)];
%   end
% end
% if length(X) > 4
%   if X(end).is_right_foot
%     goal = foot_goals.right;
%   else
%     goal = foot_goals.left;
%   end
    
%   c = biped.checkStepFeasibility(X(end-3).pos, goal, X(end-3).is_right_foot);
%   if all(c < 0)
%     X = [X(1:end-3), X(end), X(end-1)];
%   end
% end

X_orig = X;
if options.yaw_fixed
  for j = 1:length(X)
    X(j).pos_fixed(6) = 1;
  end
end
  
Xpos = [X.pos];
x_flat = reshape(Xpos([1,2,6],:), 1, []);

x_l = min(Xpos, [], 2);
x_u = max(Xpos, [], 2);
x_lb = x_l - abs(x_u - x_l) * 0.5;
x_ub = x_u + abs(x_u - x_l) * 0.5;
lb = repmat(x_lb, 1, length(X(1,:)));
ub = repmat(x_ub, 1, length(X(1,:)));
lb(6,:) = repmat(-pi, 1, length(X(1,:)));
ub(6,:) = repmat(pi, 1, length(X(1,:)));

Xfix = [X.pos_fixed];
lb(Xfix>0) = Xpos(Xfix>0);
ub(Xfix>0) = Xpos(Xfix>0);

ncon = 2 * (length(x_flat) - 3);
nvar = length(x_flat);
A = [eye(ncon/2, nvar) - [zeros(ncon/2, 3), eye(ncon/2, nvar-3)];
     -1 .* eye(ncon/2, nvar) + [zeros(ncon/2, 3), eye(ncon/2, nvar-3)]];
A = sparse(A);
b = repmat([max_diag_dist; max_diag_dist; biped.max_step_rot], ncon / 3, 1);
         

[x_flat,fval,exitflag,output,lambda,grad] = fmincon(@cost, x_flat,A,b,[],[],...
  reshape(lb([1,2,6],:), 1, []), reshape(ub([1,2,6],:), 1, []),@nonlcon,...
  optimset('Algorithm', 'interior-point', 'MaxIter', 30, 'Display', 'off', 'TolX', 0.01, 'TolCon', 0.01));
% grad
% exitflag

X = X_orig;
Xpos = [X.pos];
Xpos([1,2,6], :) = reshape(x_flat, 3, []);
Xposc = num2cell(Xpos, 1);
[X.pos] = Xposc{:};

if (exitflag == -2 && options.max_num_steps > length(X) - 2) || options.min_num_steps > length(X) - 2
  % Xpos
  c = biped.checkStepFeasibility(Xpos(:, 1:end-1), Xpos(:, 2:end), [X(1:end-1).is_right_foot]);
  c = reshape(c, 3, length(X)-1);
  c = max(c, [], 1);
  [~, ndx] = max(c);
  % locate max constraint violation
  ndx = min(ndx, length(X) - 2);
  ndx = max(ndx, 2);
  new_X(1).pos = interp1([0, 1], [X(ndx-1).pos, X(ndx+1).pos]', 0.5)';
  new_X(2).pos = interp1([0, 1], [X(ndx).pos, X(ndx+2).pos]', 0.5)';
  new_X(1).time = 0;
  new_X(2).time = 0;
  new_X(1).id = biped.getNextStepID();
  new_X(2).id = biped.getNextStepID();
  new_X(1).pos_fixed = zeros(6, 1);
  new_X(2).pos_fixed = zeros(6, 1);
  new_X(1).is_right_foot = X(ndx+1).is_right_foot;
  new_X(2).is_right_foot = X(ndx).is_right_foot;
  X = [X(1:ndx), new_X, X(ndx+1:end)];
  t = num2cell(biped.getStepTimes([X.pos]));
  [X.time] = t{:};
end
if (options.max_num_steps < length(X) && length(X) > 4)
  X = [X(1:end-4), X(end-1:end)];
end

for j = 1:size(X, 2)
  X(j).pos(3) = heightfun(X(j).pos(1:2));
end

function [c, ceq] = nonlcon(x_flat)
  X = X_orig;
  Xpos = [X.pos];
  Xpos([1,2,6], :) = reshape(x_flat, 3, []);
  % [Xright, Xleft] = biped.stepLocations(X);

%   c = [biped.checkStepFeasibility(Xpos(:, 1:end-1), Xpos(:, 2:end), [X(1:end-1).is_right_foot]);
%        biped.checkStepOrientation(Xpos)];
  c = [biped.checkStepFeasibility(Xpos(:, 1:end-1), Xpos(:, 2:end), [X(1:end-1).is_right_foot])];
  ceq = 0;
end

function c = cost(x_flat)
  X = X_orig;
  Xpos = [X.pos];
  Xpos([1,2,6], :) = reshape(x_flat, 3, []);
%   Xgoal = repmat(foot_goals.right, 1, size(Xpos, 2));
%   ndx_left = find([X.is_right_foot] == 0);
%   Xgoal(:, ndx_left) = repmat(foot_goals.left, 1, length(ndx_left));
%   c = sum(sum((Xpos - Xgoal).^2, 1), 2);
  
%   Xgoal = repmat(foot_goals.left, 1, size(Xpos, 2));
%   ndx_left = find([X.is_right_foot] == 0);
%   Xgoal(:, ndx_left) = repmat(foot_goals.right, 1, length(ndx_left));
%   c = biped.checkStepFeasibility(Xpos, Xgoal, [X.is_right_foot]);
%   c = sum(c);
  if X(end).is_right_foot
    c = sum((X(end).pos - foot_goals.right).^2) + sum((X(end-1).pos - foot_goals.left).^2);
  else
    c = sum((X(end).pos - foot_goals.left).^2) + sum((X(end-1).pos - foot_goals.right).^2);
  end
end

end