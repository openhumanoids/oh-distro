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
nt_support = length(support_times);
support_times(end+1) = support_times(end)+eps;

%% create desired joint trajectory
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(ladderInverseKinQ(r));
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
%ikoptions = ikoptions.setSequentialSeedFlag(true);
 %ikoptions = ikoptions.setMex(false);

msg ='Ladder Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);

[q_data, t, ee_info] = ladderIK(r,support_times,q0,qstar,ee_info,ladder_opts.coarse,ikoptions);

com_data = zeros(3,nt_support);
for i = 1:nt_support
  kinsol = doKinematics(r,q_data(:,i));
  com_data(:,i) = getCOM(r,kinsol);
end

ladder_opts.fine.comtraj = PPTrajectory(foh(support_times(1:end-1),com_data));
[q_data, t_data] = ladderIK(r,0:dt:tf,q0,qstar,ee_info,ladder_opts.fine,ikoptions);
x_data = [q_data;zeros(size(q_data))];

% Plot COM traj
for i = 1:size(x_data,2)
  kinsol = doKinematics(r,x_data(1:nq,i));
  com = getCOM(r,kinsol);
  com(3) = 0;
  phi = i/size(x_data,2);
  lcmgl.glColor3f(phi,0,1-phi);
  lcmgl.sphere(com,0.02,20,20);
end
lcmgl.switchBuffers();
end
