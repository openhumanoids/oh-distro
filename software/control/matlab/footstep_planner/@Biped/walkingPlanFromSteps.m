function [xtraj, qtraj, htraj, supptraj, comtraj, lfoottraj,rfoottraj, V, ts,zmptraj] = walkingPlanFromSteps(biped, x0, qstar, X)
Xpos = [X.pos];
Xright = Xpos(:, [X.is_right_foot] == 1);
Xleft = Xpos(:, [X.is_right_foot] == 0);

nq = getNumDOF(biped);
q0 = x0(1:nq);
kinsol = doKinematics(biped,q0);

[zmptraj,foottraj, supptraj] = planInitialZMPTraj(biped, q0, X);
zmptraj = setOutputFrame(zmptraj,desiredZMP);

% construct ZMP feedback controller
com = getCOM(biped,kinsol);
zmap = getTerrainHeight(biped,com(1:2));
limp = LinearInvertedPendulum(com(3)-zmap);
% get COM traj from desired ZMP traj
comtraj = ZMPplanner(limp,com(1:2),[0;0],zmptraj);

% figure(2); 
% clf; 
% subplot(2,1,1); hold on;
% fnplt(zmptraj(1));
% fnplt(comtraj(1));
% subplot(2,1,2); hold on;
% fnplt(zmptraj(2));
% fnplt(comtraj(2));

[~,V] = ZMPtracker(limp,zmptraj);

% time spacing of samples for IK
ts = 0:0.08:zmptraj.tspan(end);

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

rfoot_body = biped.findLink(biped.r_foot_name);
lfoot_body = biped.findLink(biped.l_foot_name);

msg ='Footstep Planner: Computing robot plan...'; disp(msg); send_status(3,0,0,msg);
% v = r.constructVisualizer;
% v.display_dt = 0.05;
htraj = [];
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    try
      q(:,i) = approximateIK(biped,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],foottraj.right.orig.eval(t),lfoot_body,[0;0;0],foottraj.left.orig.eval(t),options);
    catch err
      q(:,i) = inverseKin(biped,q(:,i-1),0,[comtraj.eval(t);nan],rfoot_body,[0;0;0],foottraj.right.orig.eval(t),lfoot_body,[0;0;0],foottraj.left.orig.eval(t),options);
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
