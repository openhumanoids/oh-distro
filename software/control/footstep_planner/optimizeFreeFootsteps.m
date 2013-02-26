function [X, Xright, Xleft] = optimizeFreeFootsteps(X, biped, ndx_r, ndx_l, fixed_steps)

if nargin < 5
  fixed_steps = repmat({[]}, length(X(1,:)), 2);
end

[fixed_steps{1,1}, fixed_steps{1,2}] = biped.footPositions(X(:,1));
[fixed_steps{end,1}, fixed_steps{end,2}] = biped.footPositions(X(:,end));

function c = cost(x_flat)
  X = locate_step_centers(x_flat);
  [Xright, Xleft] = biped.footPositions(X);
  [d1, r1] = stepDistance(Xright(:,1:(end-1)), Xright(:,2:end));
  [d2, r2] = stepDistance(Xleft(:,1:(end-1)), Xleft(:,2:end));
  c1 = sum(d1.^2 + (r1 .* (biped.max_step_length / biped.max_step_rot)).^2 + (d1 .* r1 .* (10 * biped.max_step_length / biped.max_step_rot)).^2);
  c2 = sum(d2.^2 + (r2 .* (biped.max_step_length / biped.max_step_rot)).^2 + (d2 .* r2 .* (10 * biped.max_step_length / biped.max_step_rot)).^2);
  c = c1 + c2;
%   figure(23);
%   clf
%   x_flat
%   quiver(Xright(1,:), Xright(2,:), cos(Xright(6,:)), sin(Xright(6,:)),'Color', 'g', 'AutoScaleFactor', 0.2);
%   hold on
%   quiver(Xleft(1,:), Xleft(2,:), cos(Xleft(6,:)), sin(Xleft(6,:)),'Color', 'r', 'AutoScaleFactor', 0.2);
%   quiver(X(1,:), X(2,:), cos(X(6,:)), sin(X(6,:)),'Color', 'k', 'AutoScaleFactor', 0.2);
%   hold off
%   d = d1 + d2;
%   r = r1 + r2;
%   dist = 
%   c = sum(dist .^ 2)
end


x_flat = reshape(X([1,2,6],:), 1, []);
X = locate_step_centers(x_flat);
x_flat = reshape(X([1,2,6],:), 1, []);

x_l = min(X, [], 2);
x_u = max(X, [], 2);
x_lb = x_l - abs(x_u - x_l) * 0.5
x_ub = x_u + abs(x_u - x_l) * 0.5

lb = [repmat(x_lb(1), 1, length(X(1,:))); repmat(x_lb(2), 1, length(X(1,:))); -pi * ones(1, length(X(1,:)))];
ub = [repmat(x_ub(1), 1, length(X(1,:))); repmat(x_ub(2), 1, length(X(1,:))); pi * ones(1, length(X(1,:)))];


x_flat = fmincon(@cost, x_flat,[],[],[],[],...
  reshape(lb, 1, []), reshape(ub, 1, []),[],...
  optimset('MaxIter', 100, 'Display', 'off'));


X = locate_step_centers(x_flat);
[Xright, Xleft] = biped.footPositions(X);
Xright = Xright(:, ndx_r);
Xleft = Xleft(:, ndx_l);


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
%   for i = 1:length(Xright(1,:))
%     if ~isempty(fixed_steps{i, 1})
%       Xright(:,i) = fixed_steps{i,1};
%     end
%     if ~isempty(fixed_steps{i, 2})
%       Xleft(:,i) = fixed_steps{i, 2};
%     end
%   end
end
end