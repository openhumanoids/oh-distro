function [x_data, ts] = robotLadderPlan(r_minimal,r, q0, qstar, tf, link_constraints,support_times,support)

nq = r.getNumDOF();

utorso = r.findLinkInd('utorso');
utorso_threshold = 10*pi/180;
hand_threshold = 10*pi/180;

% time spacing of samples for IK
dt = 0.02;
ts = 0:dt:tf;
if length(ts)>3000 % limit number of IK samples to something reasonable
  ts = linspace(0,tf,3000);
  dt = ts(2)-ts(1);
end
nt = length(ts);

%% create desired joint trajectory
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bkz = 10;
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(diag(cost(1:r.getNumDOF)));
ikoptions = ikoptions.setSequentialSeedFlag(true);
% ikoptions = ikoptions.setMex(false);

msg ='Walk Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
q = zeros(nq,nt);
q(:,1) = q0;
basic_constraints = {};
basic_constraints = [ ...
  basic_constraints
  {AllBodiesClosestDistanceConstraint(r,0.05,1e3)}];
basic_constraints = [ ...
  basic_constraints, ...
  {WorldGazeDirConstraint(r,utorso,[0;0;1],[0;0;1],utorso_threshold)}];
for i = 1:(length(support)-1)
  qsc = QuasiStaticConstraint(r.getManipulator(),[support_times(i),support_times(i+1)]);
  qsc = qsc.setShrinkFactor(1);
  qsc = qsc.setActive(true);
  for j = 1:length(support{i}.bodies)
    body_idx = support{i}.bodies(j);
    qsc = qsc.addContact(body_idx, ...
      r_minimal.getBody(body_idx).contact_pts(:,support{i}.contact_pts{j}));
    ts_idx = find(support_times(i) < ts & ts < support_times(i+1),1);
    link_idx = [link_constraints.link_ndx];
    pos = link_constraints(link_idx==body_idx).traj.eval(ts(ts_idx));
    if ~isempty(pos)
      basic_constraints = [ ...
        basic_constraints, ...
        {WorldEulerConstraint(r,body_idx, ...
        pos(4:6), ...
        pos(4:6), ...
        [support_times(i),support_times(i+1)])}];
    end
  end
  basic_constraints = [basic_constraints, {qsc}];
end
for i=2:nt
  t = ts(i);
  constraints = basic_constraints;
  for j = 1:length(link_constraints)
    if ~isempty(link_constraints(j).traj)
      pos_eq = link_constraints(j).traj.eval(t);
      constraints = [ ...
        constraints, ...
        {WorldPositionConstraint(r,link_constraints(j).link_ndx, ...
            link_constraints(j).pt(1:3), ...
            pos_eq(1:3),pos_eq(1:3),[t,t]), ...
         WorldEulerConstraint(r,link_constraints(j).link_ndx, ...
            pos_eq(4:6),pos_eq(4:6),[t,t])}];
    else
      pos_min = link_constraints(j).min_traj.eval(t);
      pos_max = link_constraints(j).max_traj.eval(t);
      constraints = [ ...
        constraints, ...
        {WorldPositionConstraint(r, link_constraints(j).link_ndx, ...
          link_constraints(j).pt, pos_min(1:3), pos_max(1:3),[t,t])}];
    end
  end
  q(:,i) = inverseKin(r,q(:,i-1),qstar,constraints{:},ikoptions);
end
% [q(:,2:end),info] = inverseKinPointwise(r,ts(2:end),repmat(q0,1,nt-1),repmat(qstar,1,nt-1),constraints{:},ikoptions);
q = q(:,1:5:end);
x_data = zeros(2*nq,size(q,2));
x_data(1:getNumDOF(r),:) = q;
ts = ts(1:5:end)

