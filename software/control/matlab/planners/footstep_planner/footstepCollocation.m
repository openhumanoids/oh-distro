function [X, exitflag, output_cost] = footstepCollocation(biped, seed_steps, goal_pos, params, safe_regions)

debug = true;
USE_SNOPT = 1;
USE_MEX = 2;

right_foot_lead = seed_steps(1).is_right_foot;

if ~isfield(params, 'nom_step_width'); params.nom_step_width = 0.26; end

st0 = seed_steps(2).pos;
goal_pos.right(6) = st0(6) + angleDiff(st0(6), goal_pos.right(6));
goal_pos.left(6) = goal_pos.right(6) + angleDiff(goal_pos.right(6), goal_pos.left(6));
goal_pos.right(3) = st0(3);
goal_pos.left(3) = st0(3);
goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
dgoal = norm(goal_pos.center(1:2) - seed_steps(1).pos(1:2));

function [c, ceq, dc, dceq] = constraints(x)
  if USE_MEX == 0
    [c, ceq, dc, dceq] = stepCollocationConstraints(x);
  elseif USE_MEX == 1
    [c, ceq, dc, dceq] = stepCollocationConstraintsMex(x);
  else
    [c, ceq, dc, dceq] = stepCollocationConstraints(x);
    [c_mex, ceq_mex, dc_mex, dceq_mex] = stepCollocationConstraintsMex(x);
    if isempty(c)
      assert(isempty(c_mex));
      assert(isempty(dc));
      assert(isempty(dc_mex));
    else
      valuecheck(c, c_mex, 1e-8);
      valuecheck(dc, dc_mex, 1e-8);
    end
    valuecheck(ceq, ceq_mex, 1e-8);
    valuecheck(dceq, dceq_mex, 1e-8);
  end
end

function [c, dc] = objfun(x)
  [steps, steps_rel] = decodeCollocationSteps(x);
  [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, right_foot_lead, dgoal, [params.nom_forward_step; params.nom_step_width]);
end

function [F,G] = collocation_userfun(x)
  [c, ceq, dc, dceq] = constraints(x);
  [cost, dCost] = objfun(x);
  F = [cost; reshape(c, [], 1); reshape(ceq, [], 1); zeros(size(A,1),1); zeros(size(Aeq,1),1)];
  G = [dCost, dc, dceq];
  G = reshape(G(iGndx), [], 1);
end

function stop = plotfun(x)
  stop = stepCollocationPlotfun(x, r_ndx, l_ndx);
end

params.forward_step = params.max_forward_step;
[A_reach, b_reach] = biped.getFootstepDiamondCons(true, params);
if length(safe_regions) > 1
  params.max_num_steps = min(params.max_num_steps, length(safe_regions));
end
min_steps = max([params.min_num_steps+1,2]);
max_steps = params.max_num_steps + 1;

steps = [seed_steps(2:end).pos];
nsteps = size(steps,2);

if ~right_foot_lead
  r_ndx = 1:2:nsteps;
  l_ndx = 2:2:nsteps;
else
  r_ndx = 2:2:nsteps;
  l_ndx = 1:2:nsteps;
end

nv = 12 * nsteps;

[A, b, Aeq, beq] = constructCollocationAb(A_reach, b_reach, nsteps, right_foot_lead, []);

lb = -inf(12,nsteps);
ub = inf(size(lb));
lb([4,5,10,11],:) = 0;
ub([4,5,10,11],:) = 0;
max_total_z_excursion = 10;
lb(3,:) = st0(3) - max_total_z_excursion;
ub(3,:) = st0(3) + max_total_z_excursion;
% Require that the first step be at the current stance foot pose
lb(1:6,1) = st0;
ub(1:6,1) = st0;
lb(7:12,1) = st0;
ub(7:12,1) = st0;
lb(7:9, :) = -2; % 2m maximum delta between steps
ub(7:9, :) = 2;
lb(12,:) = -pi;
ub(12,:) = pi;

x0 = encodeCollocationSteps(steps);

for j = 2:nsteps
  x_ndx = (j-1)*12+(1:6);
  if length(safe_regions) == 1
    region_ndx = 1;
  else
    region_ndx = j-1;
  end
  region = safe_regions(region_ndx);
  num_region_cons = length(region.b);
  expanded_A = zeros(num_region_cons, nv);
  expanded_A(1:length(region.b),x_ndx([1,2,6])) = region.A;
  A = [A; expanded_A];
  b = [b; region.b];

  expanded_Aeq = zeros(1, nv);
  expanded_Aeq(1, x_ndx([1,2,3])) = region.normal;
  Aeq = [Aeq; expanded_Aeq];
  beq = [beq; region.normal' * region.point];
end


if USE_SNOPT
  snseti ('Major Iteration limit', 250);
  if debug
    snseti ('Verify level', 3);
  else
    snseti ('Verify level', 0);
  end

  snseti ('Superbasics limit', 2000);
  n_obj = 1;
  n_proj_cons = 2*nsteps;
  iG = boolean(zeros(nv, n_obj + n_proj_cons));
  iA = boolean(zeros(size(iG,2)+size(A,1)+size(Aeq,1),nv));
  iG(:,1) = 1;

  x1_ndx = 1:6;
  dx_ndx = 7:12;
  con_ndx = (1:2)+1;
  iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
  iG(dx_ndx(1:2),con_ndx) = diag([1,1]);

  for j = 2:nsteps
    con_ndx = (j-1)*2+(1:2) + n_obj;
    x1_ndx = (j-2)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    x2_ndx = (j-1)*12+(1:6);
    iG(x2_ndx(1:2),con_ndx) = diag([1,1]);
    iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
    iG(x1_ndx(6),con_ndx) = [1,1];
    iG(dx_ndx(1:2),con_ndx) = [1 1; 1 1];
  end

%   for j = 2:nsteps
%     con_ndx = j + n_obj + n_proj_cons;
%     x1_ndx = (j-1)*12+(1:6);
%     iG(x1_ndx(1:3), con_ndx) = 1;
%   end

  iGndx = find(iG);
  [jGvar, iGfun] = find(iG);

  iA(size(iG,2)+(1:size(A,1)),:)= A ~= 0;
  iA(size(iG,2)+size(A,1)+(1:size(Aeq,1)),:)= Aeq ~= 0;
  iAndx = find(iA);
  [iAfun, jAvar] = find(iA);
  A_sn = [zeros(size(iG,2),nv); A; Aeq];

  lb = reshape(lb, [],1);
  ub = reshape(ub, [],1);
  xlow = lb;
  xupp = ub;
  xmul = zeros(size(lb));
  xstate = zeros(size(lb));
  Flow = [-inf; zeros(n_proj_cons, 1); -inf(size(A,1),1); beq];
  Fupp = [inf; zeros(n_proj_cons, 1); b; beq];
  Fmul = zeros(size(Flow));
  Fstate = Fmul;
  ObjAdd = 0;
  ObjRow = 1;
  global SNOPT_USERFUN
  SNOPT_USERFUN = @collocation_userfun;
  % tic
  [xstar, fval, ~, ~, exitflag] = snsolve(x0,xlow,xupp,xmul,xstate,    ...
               Flow,Fupp,Fmul,Fstate,      ...
               ObjAdd,ObjRow,A_sn(iAndx),iAfun,jAvar,...
               iGfun,jGvar,'snoptUserfun');
  % toc
  if debug
    exitflag
  end
else
  % tic
  [xstar, fval, exitflag] = fmincon(@objfun, x0, sparse(A), b, ...
                  sparse(Aeq), beq, lb, ub, @constraints, ...
                  optimset('Algorithm', 'interior-point', ...
                  'DerivativeCheck', 'on', ...
                  'GradConstr', 'on', ...
                  'GradObj', 'on', 'OutputFcn',{}));
  % toc
end

% plotfun(xstar);

[output_steps, output_steps_rel] = decodeCollocationSteps(xstar);
output_cost = fval(1);

if exitflag < 10
  for j = 2:nsteps
    R = rotmat(output_steps(6,j-1));
    valuecheck(output_steps(:,j-1) + [R * output_steps_rel(1:2,j); output_steps_rel(3:6,j)], output_steps(:,j),1e-4);
  end
end
% nsteps

X = seed_steps;
valuecheck(output_steps([1,2,6],1), X(2).pos([1,2,6]),1e-8);
for j = 2:nsteps
  X(j+1).pos = output_steps(:,j);
  X(j+1).is_right_foot = logical(mod(right_foot_lead+j,2));
end

end
