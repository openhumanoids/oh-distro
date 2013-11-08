function [x_data, ts] = robotLadderPlan(r_minimal,r, q0, qstar, tf, link_constraints,support_times,support)

nq = r.getNumDOF();

pelvis = r.findLinkInd('pelvis');
utorso = r.findLinkInd('utorso');
utorso_threshold = 10*pi/180;
hand_threshold = sin(5*pi/180);
pelvis_threshold = 0.05;
com_tol = 0.05;


% time spacing of samples for IK
dt = 0.1;
ts = 0:dt:tf;
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,tf,300);
  dt = ts(2)-ts(1);
end
nt = length(ts);
ts(nt+1) = ts(end)+eps;
nt_support = length(support_times);
support_times(end+1) = support_times(end)+eps;

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
constraints = {};
% basic_constraints = [ ...
%   basic_constraints
%   {AllBodiesClosestDistanceConstraint(r,0.05,1e3)}];
% constraints = [ ...
%   constraints, ...
%   {WorldGazeDirConstraint(r,utorso,[0;0;1],[0;0;1],utorso_threshold)}];
kinsol = doKinematics(r,q0);
pelvis_xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
o_T_pelvis = HT(pelvis_xyzrpy(1:3),pelvis_xyzrpy(4),pelvis_xyzrpy(5),pelvis_xyzrpy(6));
constraints = [ ...
        constraints, ...
        {WorldPositionInFrameConstraint(r,pelvis, ...
            [0;0;0], o_T_pelvis, [NaN;-pelvis_threshold;NaN], ...
            [NaN;pelvis_threshold;NaN],[ts(1),ts(end)])}];
for i=2:nt
  t = ts(i);
  for j = 1:length(link_constraints)
    if ~isempty(link_constraints(j).traj)
      pos_eq = link_constraints(j).traj.eval(t);
      constraints = [ ...
        constraints, ...
        {WorldPositionConstraint(r,link_constraints(j).link_ndx, ...
            link_constraints(j).pt(1:3), ...
            pos_eq(1:3),pos_eq(1:3),[t,ts(i+1)-eps]),...
         WorldEulerConstraint(r,link_constraints(j).link_ndx, ...
            pos_eq(4:6), ...
            pos_eq(4:6), ...
            [t,ts(i+1)-eps])}];
    else
      pos_min = link_constraints(j).min_traj.eval(t);
      pos_max = link_constraints(j).max_traj.eval(t);
      constraints = [ ...
        constraints, ...
        {WorldPositionConstraint(r, link_constraints(j).link_ndx, ...
          link_constraints(j).pt, pos_min(1:3), pos_max(1:3),[t,ts(i+1)-eps]), ...
         WorldQuatConstraint(r,link_constraints(j).link_ndx,pos_min(4:7),hand_threshold,[t,ts(i+1)-eps])}];
    end
  end
%   [q(:,i),info] = inverseKin(r,q(:,i-1),qstar,constraints{:},ikoptions);

end
qsc_cell = cell(length(support)-1,1);
qsc_idx = zeros(nt,1);
q_support_times = zeros(nq,nt_support);
q_support_times(:,1) = q0;
com_support_times = zeros(3,nt_support);
kinsol = doKinematics(r,q0);
com_support_times(:,1) = r.getCOM(kinsol);
for i = 1:(nt_support-1)
  qsc = QuasiStaticConstraint(r.getManipulator(),[support_times(i),support_times(i+1)]);
  qsc = qsc.setShrinkFactor(0.5);
  qsc = qsc.setActive(true);
  for j = 1:length(support{i}.bodies)
    body_idx = support{i}.bodies(j);
    qsc = qsc.addContact(body_idx, ...
      r_minimal.getBody(body_idx).contact_pts(:,support{i}.contact_pts{j}));
    ts_idx = find(support_times(i) < ts & ts < support_times(i+1),1);
    link_idx = [link_constraints.link_ndx];
  end
  qsc_cell{i} = qsc;
  if i > 1
    [q_this,info] = inverseKinPointwise(r,support_times(i), ...
      qstar,qstar,constraints{:},qsc,ikoptions);
    disp(info);
    
    [q_prev,info] = inverseKinPointwise(r,support_times(i), ...
      qstar,qstar,constraints{:},qsc_cell{i-1},ikoptions);
    disp(info);
    
    kinsol = doKinematics(r,q_prev);
    if qsc.checkConstraint(kinsol)
      com_support_times(:,i) = r.getCOM(kinsol);
    else
      kinsol = doKinematics(r,q_this);
      if qsc_cell{i-1}.checkConstraint(kinsol)
        com_support_times(:,i) = r.getCOM(kinsol);
      else
        error('robotLadderPlan:badSupportSequence','Invalid sequence of supports');
      end
    end
  end
  qsc_idx(qsc.isTimeValid(ts)) = i;
end
com_support_times(:,end) = com_support_times(:,end-1);
com_traj = PPTrajectory(foh(support_times(1:end-1),com_support_times));
for i=2:nt
  com = eval(com_traj,ts(i));
  com(3) = NaN;
  com_constraint = WorldCoMConstraint(r,com-com_tol,com+com_tol);
  [q(:,i),info] = inverseKinPointwise(r,ts(i),q(:,i-1),q(:,i-1),constraints{:},com_constraint,ikoptions);
  if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
  %   disp(info);
end
% [q(:,2:end),info] = inverseKinPointwise(r,ts(2:end),repmat(q0,1,nt-1),repmat(qstar,1,nt-1),constraints{:},ikoptions);
% q = q(:,1:5:end);
x_data = zeros(2*nq,size(q,2));
x_data(1:getNumDOF(r),:) = q;
ts = ts(1:end-1)
end
