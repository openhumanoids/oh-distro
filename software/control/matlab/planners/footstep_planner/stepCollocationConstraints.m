function [c, ceq, dc, dceq] = stepCollocationConstraints(x)
  [steps, rel_steps] = decodeSteps(x);
  nsteps = size(steps, 2);
  nv = length(x);
  nceq = 2 * nsteps;
  ceq = zeros(2, nsteps);
  dceq = zeros(nv, nceq);

  ceq(1:2,1) = steps(1:2,1) - rel_steps(1:2,1);
  dceq(1:2,1:2) = diag(ones(2,1));
  dceq(7:8,1:2) = diag(-ones(2,1));

  for j = 2:nsteps
    con_ndx = (j-1)*2+1:j*2;
    R = rotmat(steps(6,j-1));
    ct = R(1,1);
    st = -R(1,2);
    dxy = R * rel_steps(1:2,j);
    proj = steps(1:2,j-1) + dxy;
    ceq(:,j) = steps(1:2,j) - proj;
    x1_ndx = (j-2)*12+(1:6);
    dx_ndx = (j-1)*12+(7:12);
    x2_ndx = (j-1)*12+(1:6);
    dceq(x2_ndx(1:2),con_ndx) = diag(ones(2,1));
    dceq(x1_ndx(1:2),con_ndx) = -diag(ones(2,1));
    dx = rel_steps(1,j);
    dy = rel_steps(2,j);

    dceq(x1_ndx(6),con_ndx) = -[-dx*st - dy*ct, dx*ct - dy*st];

    dceq(dx_ndx(1),con_ndx) = -[ct,st];
    dceq(dx_ndx(2),con_ndx) = -[-st,ct];
  end
  ceq = reshape(ceq, nceq, 1);
  dceq = sparse(dceq);

  c = [];
  dc = [];
end

function [steps, rel_steps] = decodeSteps(x)
  x = reshape(x, 12, []);
  steps = x(1:6,:);
  rel_steps = x(7:12,:);
end


