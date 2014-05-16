function plan = footstepMIQP(biped, seed_steps, goal_pos, nsteps, right_foot_lead, params, safe_regions, seed)

nr = length(safe_regions);
nx = 4 * nsteps;
ns = (nsteps-1) * nr;
nf = 1;
% nvar = nx + ns + nf;
nvar = nx + ns;
x_ndx = reshape(1:nx, 4, nsteps);
s_ndx = reshape(nx + (1:ns), nr, nsteps-1);
% f_ndx = nx + ns + 1;

if nargin < 8
  seed = [];
end
x0 = nan(1, nvar);
R = cell(nsteps, 1);
if ~isempty(seed)
  sizecheck(seed.steps, [6, nan]);
  sizecheck(seed.region_assignments, [nr, nan]);
  x0(x_ndx(:,1:size(seed.steps, 2))) = seed.steps([1,2,3,6],:);
  x0(s_ndx(:,1:size(seed.region_assignments, 2))) = seed.region_assignments;
  for k = 2:nsteps
    R{k} = [rotmat(-seed.steps(6,k-1)), zeros(2,2);
     zeros(2,2), eye(2)];
  end
  start_pos = seed.steps(:,1);
else
  if right_foot_lead
    start_pos = foot_orig.left;
  else
    start_pos = foot_orig.right;
  end
  for k = 1:nsteps
    R{k} = [rotmat(-start_pos(6)), zeros(2,2);
     zeros(2,2), eye(2)];
  end
end

[A_reach, b_reach] = biped.getFootstepDiamondCons(true, params);
assert(all(all(A_reach(:,4:5) == 0)), 'Linear constraints on roll and pitch are not supported yet');
A_reach = A_reach(:,[1:3,6]);

nom_step = [0; params.nom_step_width; 0; 0];
% R = [rotmat(-start(6)), zeros(2,2);
%      zeros(2,2), eye(2)];
w_goal = 10 * [1;1;0;0;0;0];
% w_rel = 1.2 * [1/params.nom_forward_step^2;1;1;0;0;0];
w_rel = 0.8 * [1/params.nom_forward_step^2;1;1;0;0;0];


% Normalize the goal weight so that the plans don't stretch out as the goal
% gets farther away
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
dgoal = norm(goal_pos.center(1:2) - mean([foot_orig.right(1:2), foot_orig.left(1:2)], 2));
extra_distance = max(dgoal - (nsteps - 1) * params.nom_forward_step, 0.01);
w_goal(1:2) = w_goal(1:2) * sqrt(1 / (extra_distance));

if ~right_foot_lead
  r_ndx = 1:2:nsteps;
  l_ndx = 2:2:nsteps;
else
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
end


A = [];
b = [];
Aeq = [];
beq = [];
Q = zeros(nvar, nvar);
c = zeros(nvar, 1);
lb = -inf(nvar, 1);
ub = inf(nvar, 1);

for j = 2:nsteps
  Ai = zeros(size(A_reach, 1), nvar);
  if ismember(j, r_ndx)
    rA_reach = A_reach * diag([1,-1,1,-1]) * R{j};
  else
    rA_reach = A_reach * R{j};
  end
  Ai(:,x_ndx(:,j)) = rA_reach;
  Ai(:,x_ndx(:,j-1)) = -rA_reach;
  bi = b_reach;
  A = [A; Ai];
  b = [b; bi];
end

w_goal = diag(w_goal([1,2,3,6]));
for j = nsteps-1:nsteps
  Q(x_ndx(:,j), x_ndx(:,j)) = w_goal * w_goal';
  xg = reshape(goal_pos.center([1,2,3,6]), [], 1);
  c(x_ndx(:,j)) = -2 * xg' * w_goal * w_goal';
end

w_rel = diag(w_rel([1,2,3,6]));
for j = 2:nsteps
%   if j == nsteps
%     w_rel = w_rel * diag([0,0,0,1]);
%   end

  Q(x_ndx(:,j), x_ndx(:,j)) = Q(x_ndx(:,j), x_ndx(:,j)) + R{j}' * w_rel * w_rel' * R{j};
  Q(x_ndx(:,j-1), x_ndx(:,j)) = Q(x_ndx(:,j-1), x_ndx(:,j)) - R{j}' * w_rel * w_rel' * R{j};
  Q(x_ndx(:,j), x_ndx(:,j-1)) = Q(x_ndx(:,j), x_ndx(:,j-1)) - R{j}' * w_rel * w_rel' * R{j};
  Q(x_ndx(:,j-1), x_ndx(:,j-1)) = Q(x_ndx(:,j-1), x_ndx(:,j-1)) + R{j}' * w_rel * w_rel' * R{j};

  if ismember(j, r_ndx)
    nom = diag([1,-1,1,-1]) *nom_step;
  else
    nom = nom_step;
  end
  c(x_ndx(:,j)) = c(x_ndx(:,j)) - (2 * nom' * w_rel * w_rel' * R{j})';
  c(x_ndx(:,j-1)) = c(x_ndx(:,j-1)) + (2 * nom' * w_rel * w_rel' * R{j})';
end

for j = 1:nsteps-1
  Aeqi = zeros(1, nvar);
  Aeqi(1, s_ndx(:,j)) = 1;
  beqi = 1;
  Aeq = [Aeq; Aeqi];
  beq = [beq; beqi];
end

M = 1000;
Ar = zeros((nsteps-1) * sum(cellfun(@(x) size(x, 1), {safe_regions.A})), nvar);
br = zeros(size(Ar, 1), 1);
offset = 0;
for j = 2:nsteps
  for r = 1:nr
    A_region = safe_regions(r).A;
    A_region = [A_region(:,1:2), zeros(size(A_region, 1), 1), A_region(:,3)];
    A_region = [A_region;
                reshape(safe_regions(r).normal, 1, []), 0;
                -reshape(safe_regions(r).normal, 1, []), 0];
    b_region = [safe_regions(r).b;
                safe_regions(r).normal' * safe_regions(r).point;
                -safe_regions(r).normal' * safe_regions(r).point];

    Ai = zeros(size(A_region, 1), nvar);
    Ai(:,x_ndx(:,j)) = A_region;
    Ai(:,s_ndx(r,j-1)) = M;
    bi = b_region + M;
    Ar(offset + (1:size(Ai, 1)), :) = Ai;
    br(offset + (1:size(Ai, 1)), :) = bi;
    offset = offset + size(Ai, 1);
  end
end
assert(offset == size(Ar, 1));
A = [A; Ar];
b = [b; br];

lb(x_ndx(:,1)) = start_pos([1,2,3,6]);
lb(x_ndx(4,:)) = start_pos(6);
ub(x_ndx(:,1)) = start_pos([1,2,3,6]);
ub(x_ndx(4,:)) = start_pos(6);
if ~isempty(seed)
  lb(x_ndx(4,1:size(seed.steps,2))) = seed.steps(6,:) - 0.05;
  ub(x_ndx(4,1:size(seed.steps,2))) = seed.steps(6,:) + 0.05;
end

clear model params
model.A = sparse([A; Aeq]);
model.obj = c;
model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
model.rhs = [b; beq];
model.lb = lb;
model.ub = ub;
model.vtype = [repmat('C', nx, 1); repmat('B', ns, 1);];
model.Q = sparse(Q);
model.start = x0;
params = struct();
params.timelimit = 5;
params.mipgap = 3e-4;
params.outputflag = 0;

result = gurobi(model, params);
xstar = result.x;
steps = xstar(x_ndx);
steps = [steps(1:3,:); zeros(2, size(steps, 2)); steps(4,:)];
diff(steps, 1, 2)

region_assignments = reshape(xstar(s_ndx), nr, nsteps-1);
[region_order, ~] = find(abs(region_assignments - 1) < 1e-2);
assert(length(region_order) == size(region_assignments, 2));


body_idx =
if right_foot_lead

plan = FootstepPlan(

if 0
  figure(1);
  clf
  plot(steps(1,r_ndx), steps(2, r_ndx), 'bo')
  hold on
  plot(steps(1,l_ndx), steps(2,l_ndx), 'ro')
  plot(steps(1,:), steps(2,:), 'k:')
  for j = 1:length(safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A(:,1:2), safe_regions(j).b);
    k = convhull(V(:,1), V(:,2));
    patch(V(k,1), V(k,2), 'k', 'FaceAlpha', 0.2);
  end
  axis equal
end