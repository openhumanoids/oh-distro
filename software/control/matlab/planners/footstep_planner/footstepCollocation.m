function [X, foot_goals] = footstepCollocation(biped, foot_orig, goal_pos, params)

debug = false;
use_snopt = 1;
use_mex = 1;
foot_orig.right(4:5) = 0;
foot_orig.left(4:5) = 0;

params.right_foot_lead = logical(params.right_foot_lead);

if ~isfield(params, 'nom_step_width'); params.nom_step_width = 0.26; end
if ~isfield(params, 'max_line_deviation'); params.max_line_deviation = params.nom_step_width * 1.5; end

X = createOriginSteps(biped, foot_orig, params.right_foot_lead);

st0 = X(2).pos;
st0(3) = 0;
c0 = biped.footCenter2StepCenter(st0, X(2).is_right_foot, params.nom_step_width);
goal_pos.right(6) = st0(6) + angleDiff(st0(6), goal_pos.right(6));
goal_pos.left(6) = goal_pos.right(6) + angleDiff(goal_pos.right(6), goal_pos.left(6));
goal_pos.right(3) = st0(3);
goal_pos.left(3) = st0(3);

goal_pos.center = mean([goal_pos.right, goal_pos.left],2);
foot_goals = goal_pos;

function [c, ceq, dc, dceq] = constraints(x)
  cf = goal_pos.center;
  if use_mex == 0 || use_mex == 2
    [c, ceq, dc, dceq] = stepCollocationConstraints(x, c0, cf, params.max_line_deviation);
  end
  if use_mex
    [c_mex, ceq_mex, dc_mex, dceq_mex] = stepCollocationConstraintsMex(x, c0, cf, params.max_line_deviation);
    if use_mex == 2
      valuecheck(c, c_mex, 1e-8);
      valuecheck(ceq, ceq_mex, 1e-8);
      valuecheck(dc, dc_mex, 1e-8);
      valuecheck(dceq, dceq_mex, 1e-8);
    else
      c = c_mex;
      dc = dc_mex;
      ceq = ceq_mex;
      dceq = dceq_mex;
    end
  end
end

function [c, dc] = objfun(x)
  [steps, steps_rel] = decodeCollocationSteps(x);
  [c, dc] = footstepCostFun(steps, steps_rel, goal_pos, params.right_foot_lead);
end

function [F,G] = collocation_userfun(x)
  [c, ceq, dc, dceq] = constraints(x);
  [cost, dCost] = objfun(x);
  F = [cost; reshape(c, [], 1); reshape(ceq, [], 1); zeros(size(A,1),1); zeros(size(Aeq,1),1)];
  G = [dCost, dc, dceq];
  G = reshape(G(iGndx), [], 1);
end

function stop = plotfun(x)
  [steps, ~] = decodeCollocationSteps(x);
  quiver(steps(1,r_ndx), steps(2,r_ndx), cos(steps(6,r_ndx)), sin(steps(6,r_ndx)), 'g');
  hold on
  plot(steps(1,r_ndx), steps(2,r_ndx), 'g:');
  quiver(steps(1,l_ndx), steps(2,l_ndx), cos(steps(6,l_ndx)), sin(steps(6,l_ndx)), 'r');
  plot(steps(1,l_ndx), steps(2,l_ndx), 'r:');
  hold off
  axis equal
  drawnow();
  stop = false;
end

params.forward_step = params.nom_forward_step;
[A_reach_0, b_reach] = biped.getFootstepDiamondCons(true, params);
min_steps = max([params.min_num_steps+1,2]);
max_steps = params.max_num_steps + 1;

steps = [];

for nsteps = min_steps:max_steps
  A_reach = A_reach_0;
  nc = length(b_reach);
  if ~params.right_foot_lead
    r_ndx = 1:2:nsteps;
    l_ndx = 2:2:nsteps;
  else
    r_ndx = 2:2:nsteps;
    l_ndx = 1:2:nsteps;
  end
  if isempty(steps)
    steps = repmat(st0, 1, nsteps);
    steps(:,2:2:end) = repmat(biped.stepCenter2FootCenter(biped.footCenter2StepCenter(st0,~params.right_foot_lead,params.nom_step_width), params.right_foot_lead, params.nom_step_width),1,floor(nsteps/2));
  else
    steps(:,end+1) = steps(:,end-1);
  end
  nv = 12 * nsteps;

  [A, b, Aeq, beq] = constructCollocationAb(A_reach, b_reach, nsteps, params.right_foot_lead);

  lb = -inf(12,nsteps);
  ub = inf(size(lb));
  lb([3,4,5,10,11],:) = 0;
  ub([3,4,5,10,11],:) = 0;
  lb(1:6,1) = st0;
  ub(1:6,1) = st0;
  lb(7:12,1) = st0;
  ub(7:12,1) = st0;
  lb(7,end) = -0.03;
  ub(7,end) = 0.03;

  x0 = encodeCollocationSteps(steps);

  if use_snopt
    snseti ('Major Iteration limit', 250);
    if debug
      snseti ('Verify level', 3);
    else
      snseti ('Verify level', 0);
    end

    snseti ('Superbasics limit', 2000);
    iG = boolean(zeros(nv, 1+2*nsteps+nsteps));
    iA = boolean(zeros(size(iG,2)+size(A,1)+size(Aeq,1),nv));
    iG(:,1) = 1;
    x1_ndx = 1:6;
    dx_ndx = 7:12;
    con_ndx = (1:2)+1;
    iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
    iG(dx_ndx(1:2),con_ndx) = diag([1,1]);

    for j = 2:nsteps
      con_ndx = (j-1)*2+(1:2)+1+nsteps;
      x1_ndx = (j-2)*12+(1:6);
      dx_ndx = (j-1)*12+(7:12);
      x2_ndx = (j-1)*12+(1:6);
      iG(x2_ndx(1:2),con_ndx) = diag([1,1]);
      iG(x1_ndx(1:2),con_ndx) = diag([1,1]);
      iG(x1_ndx(6),con_ndx) = [1,1];
      iG(dx_ndx(1:2),con_ndx) = [1 1; 1 1];
    end

    for j = 1:nsteps
      con_ndx = j+1;
      x1_ndx = (j-1)*12+(1:6);
      iG(x1_ndx(1:2),con_ndx) = [1;1];
    end

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
    Flow = [-inf; -inf(nsteps,1); zeros(2*nsteps, 1); -inf(size(A,1),1); beq];
    Fupp = [inf; zeros(nsteps,1); zeros(2*nsteps, 1); b; beq];
    Fmul = zeros(size(Flow));
    Fstate = Fmul;
    ObjAdd = 0;
    ObjRow = 1;
    global SNOPT_USERFUN
    SNOPT_USERFUN = @collocation_userfun;
    tic
    [xstar, fval, ~, ~, exitflag] = snsolve(x0,xlow,xupp,xmul,xstate,    ...
                 Flow,Fupp,Fmul,Fstate,      ...
                 ObjAdd,ObjRow,A_sn(iAndx),iAfun,jAvar,...
                 iGfun,jGvar,'snoptUserfun');
    toc
    exitflag
  else
    tic
    [xstar, fval, exitflag] = fmincon(@objfun, x0, sparse(A), b, sparse(Aeq), beq, lb, ub, @constraints, optimset('Algorithm', 'interior-point', 'DerivativeCheck', 'on', 'GradConstr', 'on', 'GradObj', 'on', 'OutputFcn',{}));
    toc
  end

  % plotfun(xstar);

  [steps, steps_rel] = decodeCollocationSteps(xstar);
  diff_r = steps(:,r_ndx(end)) - goal_pos.right;
  diff_l = steps(:,l_ndx(end)) - goal_pos.left;
  if all(abs(diff_r) <= [0.02;0.02;0.02;0.1;0.1;0.1]) && all(abs(diff_l) <= [0.02;0.02;0.02;0.1;0.1;0.1])
    break
  end
end

for j = 2:nsteps
  R = rotmat(steps(6,j-1));
  if mod(params.right_foot_lead+j,2)
    steps_rel(6,j) = -1 * steps_rel(6,j);
  end
  valuecheck(steps(:,j-1) + [R * steps_rel(1:2,j); steps_rel(3:6,j)], steps(:,j),1e-4);
end
nsteps

valuecheck(steps([1,2,6],1), X(2).pos([1,2,6]),1e-8);
for j = 2:nsteps
  X(j+1).pos = steps(:,j);
  X(j+1).is_right_foot = logical(mod(params.right_foot_lead+j,2));
end

biped.getNextStepID(true); % reset the counter

end
