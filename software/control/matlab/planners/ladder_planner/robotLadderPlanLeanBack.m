function [x_data, t_data] = robotLadderPlanLeanBack(r, q0, qstar, comtraj, ee_info, support_times,ladder_opts)

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'robotLadderPlan');
red = {1,0,0};
blue = {0,0,1};
gray = {0.5,0.5,0.5};
black = {0,0,0};
nq = r.getNumDOF();


if ~exist('ladder_opts','var')
  ladder_opts = struct();
end
if ~isfield(ladder_opts,'coarse')
  ladder_opts.coarse = defaultLadderOptsCoarse();
end
if ~isfield(ladder_opts,'fine')
  ladder_opts.fine = defaultLadderOptsFine();
end

% time spacing of samples for IK
tf = comtraj.tspan(2);
dt = max(0.5,tf/1000);
% support_times(end+1) = support_times(end)+eps;
nt_support = length(support_times);

%% create desired joint trajectory
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(ladderInverseKinQ(r));
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
%ikoptions = ikoptions.setSequentialSeedFlag(true);
 %ikoptions = ikoptions.setMex(false);

msg ='Ladder Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);

% [q_data, t_data, ee_info] = ladderIK(r,support_times,q0,qstar,ee_info,ladder_opts.coarse,ikoptions);
% 
% com_data = zeros(3,nt_support);
% for i = 1:nt_support
%   kinsol = doKinematics(r,q_data(:,i));
%   com_data(:,i) = getCOM(r,kinsol);
% end
% ladder_opts.fine.comtraj = PPTrajectory(foh(support_times(1:end-1),com_data));
ee_info.feet(1).support_pts = r.getBodyContacts(ee_info.feet(1).idx);
ee_info.feet(2).support_pts = r.getBodyContacts(ee_info.feet(2).idx);
support_centroid = zeros(3,nt_support);
for i = 1:nt_support
  support_centroid(:,i) = computeSupportPolygonCentroid(support_times(i),ee_info.feet);
end
kinsol = doKinematics(r,q0);
com0 = r.getCOM(kinsol);
support_centroid(:,1) = com0;
ladder_opts.fine.comtraj = PPTrajectory(foh(support_times,support_centroid));

ts = 0:dt:tf;
idx_add_to_ts = false(size(support_times));
for i = 1:nt_support
  if min(abs(ts - support_times(i))) > 0.01
    idx_add_to_ts(i) = true;
  end
end
ts = sort([ts,support_times(idx_add_to_ts)]);
[q_data, t_data,~,idx_t_infeasible] = ladderIK(r,ts,q0,qstar,ee_info,ladder_opts.fine,ikoptions);
x_data = [q_data;zeros(size(q_data))];
t_data = t_data(1):dt:(length(t_data)-1)*dt;

% Plot COM traj
% v = r.constructVisualizer();
for i = 1:size(x_data,2)
%   v.draw(0,x_data(:,i));
  kinsol = doKinematics(r,x_data(1:nq,i));
  com = getCOM(r,kinsol);
  com(3) = 0;
  phi = i/size(x_data,2);
  if idx_t_infeasible(i)
    lcmgl.glColor3f(0,0,0);
  else
    lcmgl.glColor3f(phi,0,1-phi);
  end
  lcmgl.sphere(com,0.02,20,20);
end
lcmgl.switchBuffers();
end
