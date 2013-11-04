function [x_data, ts] = robotLadderPlan(r, q0, qstar, comtraj, link_constraints)

nq = r.getNumDOF();

utorso = r.findLinkInd('utorso');
utorso_threshold = 10*pi/180;

% time spacing of samples for IK
ts = 0:0.1:comtraj.tspan(end);
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,comtraj.tspan(end),300);
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
ikoptions = ikoptions.sequentialSeedFlag(true);

msg ='Walk Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
q = zeros(nq,nt);
q(:,1) = q0;
constraints = {};
constraints = [ ...
  constraints
  AllBodiesClosestDistanceConstraint(r,0.1,1e3)];
constraints = [ ...
  constraints, ...
  WorldGazeDirConstraint(r,utorso,[0;0;1],[0;0;1],utorso_threshold)];
for i=1:nt
  t = ts(i);
  for j = 1:length(link_constraints)
    if ~isempty(link_constraints(j).traj)
      constraints = [ ...
        constraints, ...
        WorldPositionConstraint(r,link_constraints(j).link_ndx, ...
          link_constraints(j).pt(1:3), ...
          link_constraints(j).traj.eval(t),  ...
          link_constraints(j).traj.eval(t),[t,t])];
      constraints = [ ...
        constraints, ...
        WorldGazeOrientConstraint(r,link_constraints(j).link_ndx, ...
          [1;0;0], ...
          link_constraints(j).pt(4:7), ...
          0, ...
          hand_threshold,[t,t])];
    else
      pos_min = link_constraints(j).min_traj.eval(t);
      pos_max = link_constraints(j).max_traj.eval(t);
      constraints = [ ...
        constraints, ...
        WorldPositionConstraint(r, link_constraints(j).link_ndx, ...
          link_constraints(j).pt, pos_min, pos_max,[t,t])];
    end
  end
  constraints = [ ...
    constraints, ...
    WorldCoMConstraint(r.getMexModelPtr, ...
      [comtraj.eval(t);nan], ...
      [comtraj.eval(t);nan],[t,t])];
end
[q(:,2:end),info] = inverseKinPointwise(r,ts,[],qstar,constraints{:},ikoptions);
x_data = zeros(2*nq,nt);
x_data(1:getNumDOF(r),:) = q;

