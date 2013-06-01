function [xtraj, qtraj, htraj, support_times, supports, comtraj, link_constraints, V, ts,zmptraj] = walkingPlanFromSteps(biped, x0, qstar, footsteps, footstep_opts, fixed_links)

nq = getNumDOF(biped);
q0 = x0(1:nq);
kinsol = doKinematics(biped,q0);

[zmptraj,foottraj, support_times, supports] = planInitialZMPTraj(biped, q0, footsteps, footstep_opts);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

%% Convert the foottraj to an easier format to hand to IK
link_constraints(1) = struct('link_ndx', find(strcmp(biped.getLinkNames(),biped.foot_bodies.right.linkname),1), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', foottraj.right.orig);
link_constraints(2) = struct('link_ndx', find(strcmp(biped.getLinkNames(),biped.foot_bodies.left.linkname),1), 'pt', [0;0;0], 'min_traj', [], 'max_traj', [], 'traj', foottraj.left.orig);
for f = {'right', 'left'}
  foot = f{1};
  for g = {'toe', 'heel'}
    grp = g{1};
    for pt_ndx = biped.foot_bodies.(foot).collision_group{strcmp(biped.foot_bodies.(foot).collision_group_name, grp)}
      link_constraints(end+1) = struct('link_ndx', find(strcmp(biped.getLinkNames(),biped.foot_bodies.(foot).linkname),1), 'pt', biped.foot_bodies.(foot).contact_pts(:,pt_ndx), 'min_traj', foottraj.(foot).(grp).min, 'max_traj', foottraj.(foot).(grp).max, 'traj', []);
    end
  end
end


%% Allow the user to fix the current position of some links (useful for walking while holding the hand still, e.g.)
for j = 1:length(fixed_links)
  if isa(fixed_links(j).link, 'RigidBody')
    link_ndx = find(strcmp(biped.getLinkNames(), fixed_links(j).link.linkname),1);
  else
    link_ndx = fixed_links(j);
  end
  pos = biped.forwardKin(kinsol, link_ndx, fixed_links(j).pt,0);
  link_constraints(end+1) = struct('link_ndx', link_ndx, 'pt',fixed_links(j).pt, 'min_traj', [], 'max_traj', [], 'traj', ConstantTrajectory(pos));
end

%% construct ZMP feedback controller
com = getCOM(biped,kinsol);
zmap = getTerrainHeight(biped,com(1:2));
limp = LinearInvertedPendulum(com(3)-zmap);

% get COM traj from desired ZMP traj
[c,V] = ZMPtracker(limp,zmptraj);
comtraj = COMplanFromTracker(limp,com(1:2),[0;0],zmptraj.tspan,c);

% comf = mean([footsteps(end).pos(1:2), footsteps(end-1).pos(1:2)], 2);
% comtraj = ZMPplan(limp,com(1:2),comf,zmptraj);
% V = struct('S', [], 's1', [], 's2', []);

% time spacing of samples for IK
ts = 0:0.05:zmptraj.tspan(end);
if length(ts)>300 % limit number of IK samples to something reasonable
  ts = linspace(0,zmptraj.tspan(end),300);
end


%% create desired joint trajectory
cost = Point(biped.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_lbz = 10;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:biped.getNumDOF));
%   options.q_nom = q0;
options.q_nom = qstar;
% [options.jointLimitMin, options.jointLimitMax] = biped.getJointLimits();
% joint_names = biped.getStateFrame.coordinates(1:biped.getNumDOF());
% knee_ind = find(~cellfun(@isempty,strfind(joint_names,'kny')));
% options.jointLimitMin(knee_ind) = 0.6;

rfoot_body = biped.findLink(biped.r_foot_name);
lfoot_body = biped.findLink(biped.l_foot_name);

msg ='Walk Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
htraj = [];
full_IK_calls = 0;
length(ts)
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    approx_args = {};
    ik_args = {};
    for j = 1:length(link_constraints)
      if ~isempty(link_constraints(j).traj)
        pos_min = link_constraints(j).traj.eval(t);
        pos_max = pos_min;
      else
        pos_min = link_constraints(j).min_traj.eval(t);
        pos_max = link_constraints(j).max_traj.eval(t);
      end
      approx_args(end+1:end+3) = {link_constraints(j).link_ndx, link_constraints(j).pt, struct('min', pos_min, 'max', pos_max)};
      ik_args(end+1:end+6) = horzcat(approx_args(end-2:end), {[],[],[]});
    end
    try
      q(:,i) = approximateIK(biped,q(:,i-1),0,[comtraj.eval(t);nan],approx_args{:},options);
    catch err
      disp(err)
      full_IK_calls = full_IK_calls + 1
      q(:,i) = inverseKin(biped,q(:,i-1),0,[comtraj.eval(t);nan],[],[],[],ik_args{:},options);
    end

  else
    q = q0;
  end
  com = getCOM(biped,q(:,i));
  htraj = [htraj com(3)];
%   v.draw(t,q(:,i));
end
qtraj = PPTrajectory(spline(ts,q));
htraj = PPTrajectory(spline(ts,htraj));
xtraj = zeros(getNumStates(biped),length(ts));
xtraj(1:getNumDOF(biped),:) = q;
