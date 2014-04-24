function [A, b, Aeq, beq, step_map] = constructCollocationAb(A_reach, b_reach, nsteps, right_foot_lead, corridor_pts)

  nc = length(b_reach);
  nv = 12 * nsteps;

  step_map.ineq = containers.Map('KeyType', 'int32', 'ValueType', 'any');
  step_map.eq = containers.Map('KeyType', 'int32', 'ValueType', 'any');

  A = zeros(nc*(nsteps-1), nv);
  b = zeros(nc*(nsteps-1), 1);
  if right_foot_lead 
    A_reach = A_reach * diag([1,-1,1,1,1,-1]);
  end
  for j = 2:nsteps
    con_ndx = nc*(j-2)+1:nc*(j-1);
    var_ndx = (j-1)*12+7:j*12;
    A(con_ndx,var_ndx) = A_reach;
    b(con_ndx) = b_reach;
    A_reach = A_reach * diag([1,-1,1,1,1,-1]);
    step_map.ineq(j) = con_ndx;
  end

  if ~isempty(corridor_pts)
    [A_corr, b_corr] = poly2lincon(corridor_pts(1,:), corridor_pts(2,:));
    A_corr_full = zeros(length(b_corr) * (nsteps-1), nv);
    b_corr_full = zeros(length(b_corr) * (nsteps-1), 1);
    for j = 2:nsteps
      con_ndx = length(b_corr)*(j-2)+(1:length(b_corr));
      var_ndx = (j-1)*12+(1:6);
      A_corr_full(con_ndx, var_ndx(1:2)) = A_corr;
      b_corr_full(con_ndx) = b_corr;
    end
    A = [A;
         A_corr_full];
    b = [b; b_corr_full];
  end

  Aeq = zeros(4*(nsteps-1),nv);
  beq = zeros(4*(nsteps-1),1);
  for j = 2:nsteps
    con_ndx = (j-2)*4+(1:4);
    x1_ndx = (j-2)*12+(1:6);
    x2_ndx = (j-1)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    Aeq(con_ndx, x1_ndx(3:6)) = -diag(ones(4,1));
    Aeq(con_ndx, x2_ndx(3:6)) = diag(ones(4,1));
    Aeq(con_ndx, dx_ndx(3:6)) = -diag(ones(4,1));
    step_map.eq(j) = con_ndx;
  end
end