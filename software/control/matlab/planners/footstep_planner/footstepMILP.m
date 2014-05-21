function plan = footstepMILP(biped, seed_plan, weights, goal_pos, min_num_steps, max_num_steps)

seed_plan.sanity_check();

nsteps = length(seed_plan.footsteps); % ignore the first fixed footstep since it has no effect
nr = length(seed_plan.safe_regions);
nx = 4 * nsteps;
ns = (nsteps) * nr;
nt = nsteps;
n_goal_obj = 2;
n_rel_obj = 4 * nsteps;
nvar = nx + ns + nt + n_goal_obj + n_rel_obj;
x_ndx = reshape(1:nx, 4, nsteps);
s_ndx = reshape(nx + (1:ns), nr, ns / nr);
t_ndx = reshape(nx + ns + (1:nt), 1, nsteps);
goal_obj_ndx = nx + ns + nt + (1:n_goal_obj);
rel_obj_ndx = reshape(nx + ns + nt + n_goal_obj + (1:n_rel_obj), 4, nsteps);
start_pos = seed_plan.footsteps(2).pos;
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);

x0 = nan(1, nvar);
R = cell(nsteps, 1);
for j = 1:nsteps
  if ~any(isnan(seed_plan.footsteps(j).pos))
    x0(x_ndx(:,j)) = seed_plan.footsteps(j).pos([1,2,3,6]);
    if j > 2
      R{j} = [rotmat(-seed_plan.footsteps(j-1).pos(6)), zeros(2,2);
           zeros(2,2), eye(2)];
    end
  else
    if j > 2
      R{j} = [rotmat(-start_pos(6)), zeros(2,2);
       zeros(2,2), eye(2)];
    end
    x0(x_ndx(4,j)) = start_pos(6);
  end

  if j > 2 && ~isnan(seed_plan.region_order(j))
    ra = false(nr, 1);
    ra(seed_plan.region_order(j)) = true;
    x0(s_ndx(:,j)) = ra;
  end
end

% nom_step = [seed_plan.params.nom_forward_step; seed_plan.params.nom_step_width; 0; 0]
nom_step = [0; seed_plan.params.nom_step_width; 0; 0];
w_trim = weights.relative(1) * seed_plan.params.nom_forward_step^2;

A = [];
b = [];
Aeq = [];
beq = [];
Q = zeros(nvar, nvar);
c = zeros(nvar, 1);
lb = -inf(nvar, 1);
ub = inf(nvar, 1);

for j = 3:nsteps
  [A_reach, b_reach] = biped.getReachabilityPolytope(seed_plan.footsteps(j-1).body_idx, seed_plan.footsteps(j).body_idx, seed_plan.params);
  A_reach = A_reach(:,[1:3,6]);
  Ai = zeros(size(A_reach, 1), nvar);
  rA_reach = A_reach * R{j};
  Ai(:,x_ndx(:,j)) = rA_reach;
  Ai(:,x_ndx(:,j-1)) = -rA_reach;
  bi = b_reach;
  A = [A; Ai];
  b = [b; bi];
end

% Require that t(j) <= t(j+1)
At = zeros(nsteps-1, nvar);
At(:,t_ndx(1:end-1)) = diag(ones(nsteps-1, 1));
At(:,t_ndx(2:end)) = At(:,t_ndx(2:end)) + diag(-ones(nsteps-1, 1));
bt = zeros(size(At, 1), 1);
A = [A; At];
b = [b; bt];

% If t(j) is true, then require that step(i) == step(end) or step(end-1) as
% appropriate.
M = 1000;
for j = 3:nsteps-2
  Ati = zeros(3, nvar);
  Ati(:,x_ndx(1:3,j)) = diag(ones(3, 1));
  if mod(nsteps - j, 2)
    Ati(:,x_ndx(1:3,end-1)) = diag(-ones(3,1));
  else
    Ati(:,x_ndx(1:3,end)) = diag(-ones(3,1));
  end
  Ati = [Ati; -Ati];

  Ati(:,t_ndx(j)) = M;
  bti = M + zeros(size(Ati, 1), 1);
  A = [A; Ati];
  b = [b; bti];
end

for j = 3:nsteps
  c(t_ndx(j)) = -w_trim;
end

A_goal_obj = zeros(2 * n_goal_obj, nvar);
A_goal_obj(:, x_ndx(1:2,end)) = [eye(2); -eye(2)];
A_goal_obj(:, goal_obj_ndx) = [-eye(2); -eye(2)];
if seed_plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  b_goal_obj = [goal_pos.right(1:2); -goal_pos.right(1:2)];
else
  b_goal_obj = [goal_pos.left(1:2); -goal_pos.left(1:2)];
end
A = [A; A_goal_obj];
b = [b; b_goal_obj];
c(goal_obj_ndx) = weights.goal(1:2);

% w_rel = diag(weights.relative([1,2,3,6]));
% for j = 3:nsteps
%   if j == nsteps
%     w_rel = diag(weights.relative_final([1,2,3,6]));
%   end
%   Q(x_ndx(:,j), x_ndx(:,j)) = Q(x_ndx(:,j), x_ndx(:,j)) + R{j}' * w_rel * R{j};
%   Q(x_ndx(:,j-1), x_ndx(:,j)) = Q(x_ndx(:,j-1), x_ndx(:,j)) - R{j}' * w_rel * R{j};
%   Q(x_ndx(:,j), x_ndx(:,j-1)) = Q(x_ndx(:,j), x_ndx(:,j-1)) - R{j}' * w_rel * R{j};
%   Q(x_ndx(:,j-1), x_ndx(:,j-1)) = Q(x_ndx(:,j-1), x_ndx(:,j-1)) + R{j}' * w_rel * R{j};
% 
%   if seed_plan.footsteps(j).body_idx == Footstep.atlas_foot_bodies_idx.right
%     nom = diag([1,-1,1,-1]) *nom_step;
%   else
%     nom = nom_step;
%   end
%   c(x_ndx(:,j)) = c(x_ndx(:,j)) - (2 * nom' * w_rel * R{j})';
%   c(x_ndx(:,j-1)) = c(x_ndx(:,j-1)) + (2 * nom' * w_rel * R{j})';
% end

w_rel = weights.relative([1,2,3,6]);
for j = 3:nsteps
  if seed_plan.footsteps(j).body_idx == Footstep.atlas_foot_bodies_idx.right
    nom = diag([1,-1,1,-1]) *nom_step;
  else
    nom = nom_step;
  end
  A_rel_obj = zeros(8, nvar);
  A_rel_obj(:, x_ndx(:,j)) = [R{j}; -R{j}];
  A_rel_obj(:, x_ndx(:,j-1)) = [-R{j}; R{j}];
  A_rel_obj(:, rel_obj_ndx(:,j)) = [-eye(4); -eye(4)];
  b_rel_obj = [nom; -nom];
  A = [A; A_rel_obj];
  b = [b; b_rel_obj];
  c(rel_obj_ndx(:,j)) = w_rel;
end

for j = 3:nsteps
  Aeqi = zeros(1, nvar);
  Aeqi(1, s_ndx(:,j)) = 1;
  beqi = 1;
  Aeq = [Aeq; Aeqi];
  beq = [beq; beqi];
end

M = 1000;
Ar = zeros((nsteps-2) * sum(cellfun(@(x) size(x, 1), {seed_plan.safe_regions.A})), nvar);
br = zeros(size(Ar, 1), 1);
offset = 0;
for j = 3:nsteps
  for r = 1:nr
    A_region = seed_plan.safe_regions(r).A;
    A_region = [A_region(:,1:2), zeros(size(A_region, 1), 1), A_region(:,3)];
    A_region = [A_region;
                reshape(seed_plan.safe_regions(r).normal, 1, []), 0;
                -reshape(seed_plan.safe_regions(r).normal, 1, []), 0];
    b_region = [seed_plan.safe_regions(r).b;
                seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point;
                -seed_plan.safe_regions(r).normal' * seed_plan.safe_regions(r).point];

    Ai = zeros(size(A_region, 1), nvar);
    Ai(:,x_ndx(:,j)) = A_region;
    Ai(:,s_ndx(r,j)) = M;
    bi = b_region + M;
    Ar(offset + (1:size(Ai, 1)), :) = Ai;
    br(offset + (1:size(Ai, 1)), :) = bi;
    offset = offset + size(Ai, 1);
  end
end
assert(offset == size(Ar, 1));
A = [A; Ar];
b = [b; br];

seed_poses = [seed_plan.footsteps.pos];
lb(x_ndx(:,1:2)) = seed_poses([1,2,3,6],1:2);
lb(x_ndx(4,:)) = x0(x_ndx(4,:)) - 0.05;
ub(x_ndx(:,1:2)) = seed_poses([1,2,3,6],1:2);
ub(x_ndx(4,:)) = x0(x_ndx(4,:)) + 0.05;
lb(s_ndx(:,1:2)) = [1, 1; zeros(nr-1, 2)];
ub(s_ndx(:,1:2)) = lb(s_ndx(:,1:2));
ub(t_ndx(1:2)) = 0;
lb(t_ndx(1:2)) = 0;
ub(t_ndx(end)) = 1;
lb(t_ndx(end)) = 1;
lb(rel_obj_ndx(:,1:2)) = 0;
ub(rel_obj_ndx(:,1:2)) = 0;

% if seed_plan.footsteps(end).body_idx == Footstep.atlas_foot_bodies_idx.right
%   lb(x_ndx(1:2,end)) = goal_pos.right(1:2);
% else
%   lb(x_ndx(1:2,end)) = goal_pos.left(1:2);
% end
% ub(x_ndx(1:2,end)) = lb(x_ndx(1:2,end));

clear model params
model.A = sparse([A; Aeq]);
model.obj = c;
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.rhs = [b; beq];
model.lb = lb;
model.ub = ub;
model.vtype = [repmat('C', nx, 1); repmat('B', ns, 1); repmat('B', nt, 1); repmat('C', n_goal_obj + n_rel_obj, 1)];
% model.Q = sparse(Q);
model.start = x0;
params = struct();
params.timelimit = 5;
params.mipgap = 3e-4;
params.outputflag = 1;

result = gurobi(model, params);
xstar = result.x;
steps = xstar(x_ndx);
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];
% diff(steps, 1, 2)

region_assignments = reshape(xstar(s_ndx), nr, nsteps);
[region_order, ~] = find(abs(region_assignments - 1) < 1e-2);
assert(length(region_order) == size(region_assignments, 2));

plan = seed_plan;
for j = 1:nsteps
  plan.footsteps(j).pos = steps(:,j);
end
plan.region_order = region_order;

trim = xstar(t_ndx);
final_steps = find(trim, 2);
if plan.footsteps(end).body_idx == biped.foot_bodies_idx.right
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(end).pos(6), goal_pos.left(6)));
end
final_step_idx = min(nsteps, final_steps(end) + ceil(2 * dtheta / (pi/8)));

if plan.footsteps(1).body_idx == biped.foot_bodies_idx.right
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.right(6)));
else
  dtheta = abs(angleDiff(plan.footsteps(1).pos(6), goal_pos.left(6)));
end
% TODO: don't hardcode the turning rate here
min_num_steps = max(min_num_steps, ceil(2 * dtheta / (pi/8) + 2));


final_nsteps = min(max_num_steps, max(min_num_steps, final_step_idx));
plan = plan.slice(1:final_nsteps);

if 0
  right_foot_lead = plan.footsteps(1).body_idx == Footstep.atlas_foot_bodies_idx.right;
  if ~right_foot_lead
    r_ndx = 1:2:nsteps;
    l_ndx = 2:2:nsteps;
  else
    r_ndx = 2:2:nsteps;
    l_ndx = 1:2:nsteps;
  end
  figure(1);
  clf
  plot(steps(1,r_ndx), steps(2, r_ndx), 'bo')
  hold on
  plot(steps(1,l_ndx), steps(2,l_ndx), 'ro')
  plot(steps(1,:), steps(2,:), 'k:')
  for j = 1:length(seed_plan.safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(seed_plan.safe_regions(j).A(:,1:2), seed_plan.safe_regions(j).b);
    k = convhull(V(:,1), V(:,2));
    patch(V(k,1), V(k,2), 'k', 'FaceAlpha', 0.2);
  end
  axis equal
end