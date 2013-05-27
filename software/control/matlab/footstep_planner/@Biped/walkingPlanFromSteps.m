function [xtraj, qtraj, htraj, supptraj, comtraj, lfoottraj,rfoottraj, V, ts,zmptraj] = walkingPlanFromSteps(biped, x0, qstar, footsteps, footstep_opts)

nq = getNumDOF(biped);
q0 = x0(1:nq);
kinsol = doKinematics(biped,q0);

[zmptraj,foottraj, supptraj] = planInitialZMPTraj(biped, q0, footsteps, footstep_opts);

zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
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
ts = 0:0.08:zmptraj.tspan(end);
if length(ts)>200 % limit number of IK samples to something reasonable
  ts = linspace(0,zmptraj.tspan(end),200);
end

% create desired joint trajectory
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
% r_link_ndx = find(strcmp(biped.getLinkNames, biped.r_foot_name));
% l_link_ndx = find(strcmp(biped.getLinkNames, biped.l_foot_name));

msg ='Walk Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
htraj = [];
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    rpos = foottraj.right.orig.eval(t);
    lpos = foottraj.left.orig.eval(t);
%     support = supptraj.eval(t);
%     if ~support(r_link_ndx)
% %       rpos = rpos(1:3);
%       rpos(4:6) = nan;
%     end
%     if ~support(l_link_ndx)
% %       lpos = lpos(1:3);
%       lpos(4:6) = nan;
%     end

    try
      q(:,i) = approximateIK(biped,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],rpos,...
        lfoot_body,[0;0;0],lpos,options);
    catch err
      q(:,i) = inverseKin(biped,q(:,i-1),0,[comtraj.eval(t);nan],[],[],[],...
        rfoot_body,[0;0;0],rpos,[],[],[],...
        lfoot_body,[0;0;0],lpos,[],[],[],options);
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

lfoottraj = foottraj.left.orig;
rfoottraj = foottraj.right.orig;
