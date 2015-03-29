function link_constraints = extractLinkTrajectories(r, qtraj, body_ids)

link_constraints = struct('link_ndx', cell(1, numel(body_ids)),...
                          'pt', cell(1, numel(body_ids)),...
                          'ts', cell(1, numel(body_ids)),...
                          'coefs', cell(1, numel(body_ids)),...
                          'toe_off_allowed', cell(1, numel(body_ids)));

ts = qtraj.getBreaks();
body_poses = zeros(6, numel(ts), numel(body_ids));

for j = 1:numel(ts)
  kinsol = doKinematics(r, qtraj.eval(ts(j)));
  for k = 1:numel(body_ids)
    body_poses(:,j,k) = forwardKin(r, kinsol, body_ids(k), [0;0;0], 1);
  end
end

for j = 1:numel(body_ids)
  link_constraints(j).link_ndx = body_ids(j);
  link_constraints(j).pt = [0;0;0];
  link_constraints(j).ts = ts;
  pp = pchip(ts, body_poses(:,:,j));
  [~, coefs, l, k, d] = unmkpp(pp);
  link_constraints(j).coefs = reshape(coefs, [d, l, k]);
  link_constraints(j).toe_off_allowed = false(1, numel(ts));
end
