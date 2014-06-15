function plan = footstepMISOCP(biped, seed_plan, weights, goal_pos, min_num_steps, max_num_steps)

seed_plan.sanity_check();
rangecheck(seed_plan.footsteps(1).pos(6), -pi, pi);
rangecheck(seed_plan.footsteps(2).pos(6), -pi, pi);

nsteps = length(seed_plan.footsteps);

x = sdpvar(4, nsteps, 'full');
cos_yaw = sdpvar(1, nsteps, 'full');
sin_yaw = sdpvar(1, nsteps, 'full');
yaw = x(4,:);

cos_boundaries = reshape(bsxfun(@plus, [-2*pi:pi:2*pi; -2*pi:pi:2*pi], [-(pi/2-1); (pi/2-1)]), 1, []);
sin_boundaries = reshape(bsxfun(@plus, [-2*pi:pi:2*pi; -2*pi:pi:2*pi], [-1; 1]), 1, []);
cos_sector = binvar(length(cos_boundaries) - 1, nsteps, 'full');
sin_sector = binvar(length(sin_boundaries) - 1, nsteps, 'full');

trim = binvar(1, nsteps, 'full');
region = binvar(length(seed_plan.safe_regions), nsteps, 'full');


foci = [[.35;-0.26], [-.25;-0.26]];
ellipse_l = sum(sqrt(sum(diff(foci, [], 2).^2))) * 1.125;

seed_steps = seed_plan.step_matrix();
Constraints = [x(:,1) == seed_steps([1,2,3,6],1),...
               x(:,2) == seed_steps([1,2,3,6],2),...
               yaw >= -2 * pi,...
               yaw <= 2 * pi,...
               x(1:3,:) >= -100 + repmat(seed_steps(1:3,1), 1, nsteps),...
               x(1:3,:) <= 100 + repmat(seed_steps(1:3,1), 1, nsteps)...
               -1 <= cos_yaw <= 1,...
               -1 <= sin_yaw <= 1,...
               region(1,1:2) == 1];
             

for j = 1:nsteps
  for s = 1:length(cos_boundaries) - 1
    th0 = cos_boundaries(s);
    th1 = cos_boundaries(s+1);
    
    th = (th0 + th1)/2;
    cos_slope = -sin(th);
    cos_intercept = cos(th) - (cos_slope * th);
    Constraints = [Constraints,...
                   implies(cos_sector(s, j), th0 <= yaw(j) <= th1),...
                   implies(cos_sector(s, j), cos_yaw(j) == cos_slope * yaw(j) + cos_intercept)];
  end
  
  Constraints = [Constraints, ...
                 sum(cos_sector(:,j)) == 1,...
                 sum(region(:,j)) == 1,...
                 ];
end

for j = 1:nsteps
  for s = 1:length(sin_boundaries) - 1
    th0 = sin_boundaries(s);
    th1 = sin_boundaries(s+1);
    
    th = (th0 + th1)/2;
    sin_slope = cos(th);
    sin_intercept = sin(th) - (sin_slope * th);
    Constraints = [Constraints,...
                   implies(sin_sector(s, j), th0 <= yaw(j) <= th1),...
                   implies(sin_sector(s, j), sin_yaw(j) == sin_slope * yaw(j) + sin_intercept)];
  end
  
  Constraints = [Constraints, ...
                 sum(sin_sector(:,j)) == 1,...
                 sum(region(:,j)) == 1,...
                 ];
end    

for j = 3:nsteps
  if seed_plan.footsteps(j).body_idx == biped.foot_bodies_idx.left
    rel_foci = [foci(1,:); -foci(2,:)];
    Constraints = [Constraints, 0 <= yaw(j) - yaw(j-1) <= pi/8];
    for k = 1:size(cos_sector, 1) - 1
      Constraints = [Constraints, implies(cos_sector(k, j-1), cos_sector(k,j) | cos_sector(k+1,j)),...
                                  implies(sin_sector(k, j-1), sin_sector(k,j) | sin_sector(k+1,j))];
    end
  else
    rel_foci = foci;
    Constraints = [Constraints, -pi/8 <= yaw(j) - yaw(j-1) <= 0];
    for k = 2:size(cos_sector, 1)
      Constraints = [Constraints, implies(cos_sector(k, j-1), cos_sector(k-1,j) | cos_sector(k,j)),...
                                  implies(sin_sector(k, j-1), sin_sector(k-1,j) | sin_sector(k,j))];
    end
  end
  expr = 0;
  for k = 1:size(rel_foci, 2)
    expr = expr + norm(x(1:2,j-1) + [cos_yaw(j-1), -sin_yaw(j-1); sin_yaw(j-1), cos_yaw(j-1)] * rel_foci(:,k) - x(1:2,j));
  end
  Constraints = [Constraints,...
                 expr <= ellipse_l, ...
                 trim(j) >= trim(j-1)];
  
end

for j = 3:nsteps
  for r = 1:length(seed_plan.safe_regions)
    Ar = [seed_plan.safe_regions(r).A(:,1:2), zeros(size(seed_plan.safe_regions(r).A, 1), 1), seed_plan.safe_regions(r).A(:,3)];
    Constraints = [Constraints, implies(region(r,j), Ar * x(:,j) <= seed_plan.safe_regions(r).b)];
  end
end

for j = 2:(nsteps-2)
  if seed_plan.footsteps(j).body_idx == seed_plan.footsteps(end).body_idx
    Constraints = [Constraints, implies(trim(j), x(:,j) == x(:,end))];
  else
    Constraints = [Constraints, implies(trim(j), x(:,j) == x(:,end-1))];
  end
end

w_goal = diag(weights.goal([1,2,3,6]));
w_rel = diag(weights.relative([1,2,3,6]));
w_trim = w_rel(1) * seed_plan.params.nom_forward_step^2;

if seed_plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  goal = goal_pos.right([1,2,3,6]);
else
  goal = goal_pos.left([1,2,3,6]);
end
Objective = (x(:,nsteps) - goal)' * w_goal * (x(:,nsteps) - goal);
for j = 2:nsteps
  Objective = Objective + (x(:,j) - x(:,j-1))' * w_rel * (x(:,j) - x(:,j-1)) + -1 * w_trim * trim(j);
end
  
solvesdp(Constraints, Objective, sdpsettings('solver', 'gurobi'));

x = double(x);
yaw = double(yaw);
cos_yaw = double(cos_yaw);
sin_yaw = double(sin_yaw);
trim = double(trim);
region = double(region);
steps = zeros(6,nsteps);
steps([1,2,3,6],:) = x;

plan = seed_plan;
[region_order, ~] = find(abs(region - 1) < 1e-2);
assert(length(region_order) == size(region, 2));

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = Point(plan.footsteps(j).frames.center, steps(:,j));
end
plan.region_order = region_order;

final_steps = find(trim, 2);
if plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.left(6)));
end
final_step_idx = min(nsteps, final_steps(end) + ceil(2 * dtheta / (pi/8)));


final_nsteps = min(max_num_steps, max(min_num_steps, final_step_idx));
plan = plan.slice(1:final_nsteps);

if 0
  figure(2)
  clf
  hold on
  for j = 1:nsteps
    if seed_plan.footsteps(j).body_idx ~= biped.foot_bodies_idx.left
      rel_foci = [foci(1,:); -foci(2,:)];
    else
      rel_foci = foci;
    end
    R = [cos_yaw(j), -sin_yaw(j); sin_yaw(j), cos_yaw(j)];
    step_foci = bsxfun(@plus, R * rel_foci, steps(1:2,j));
    [X, Y] = meshgrid(linspace(min(step_foci(1,:) - 0.5), max(step_foci(1,:) + 0.5)),...
                      linspace(min(step_foci(2,:) - 0.5), max(step_foci(2,:) + 0.5)));
    Z = zeros(1, size(X,1) * size(X,2));
    for k = 1:size(step_foci, 2)
      Z = Z + sqrt(sum(bsxfun(@minus, [reshape(X, 1, []); reshape(Y, 1, [])], step_foci(:,k)).^2, 1));
    end
    Z = reshape(Z, size(X));
    contour(X,Y,Z,[ellipse_l, ellipse_l]);
    plot(step_foci(1,[1:end, 1]), step_foci(2,[1:end, 1]), 'b.-')
  end

  plot(steps(1,:), steps(2,:), 'k:')

  r_ndx = find([plan.footsteps.body_idx] == biped.foot_bodies_idx.right);
  l_ndx = find([plan.footsteps.body_idx] == biped.foot_bodies_idx.left);
  plot(steps(1,l_ndx), steps(2,l_ndx), 'ro')
  plot(steps(1,r_ndx), steps(2,r_ndx), 'go')
  quiver(steps(1,l_ndx), steps(2,l_ndx), cos(steps(6,l_ndx)), sin(steps(6,l_ndx)), 'r', 'AutoScaleFactor', 0.2);
  quiver(steps(1,r_ndx), steps(2,r_ndx), cos(steps(6,r_ndx)), sin(steps(6,r_ndx)), 'g', 'AutoScaleFactor', 0.2);
  quiver(steps(1,:), steps(2,:), cos_yaw, sin_yaw, 'k', 'AutoScaleFactor', 0.2);

  for j = 1:length(seed_plan.safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(seed_plan.safe_regions(j).A(:,1:2), seed_plan.safe_regions(j).b);
    k = convhull(V(:,1), V(:,2));
    patch(V(k,1), V(k,2), 'k', 'FaceAlpha', 0.2);
  end

  axis equal
end

