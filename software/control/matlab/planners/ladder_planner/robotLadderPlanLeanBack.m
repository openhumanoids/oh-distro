function [x_data, t_data] = robotLadderPlanLeanBack(r, q0, qstar, comtraj, ee_info, support_times,ladder_opts)

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'robotLadderPlan');
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
dt = tf/75;
nt_support = length(support_times);

%% create desired joint trajectory
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(ladderInverseKinQ(r));
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMajorIterationsLimit(5000);

msg ='Ladder Plan: Computing robot plan...'; disp(msg); send_status(6,0,0,msg);

ts = 0:dt:tf;

[q_data, t_data,~,idx_t_infeasible] = ladderIK(r,ts,q0,qstar,ee_info,ladder_opts.fine,ikoptions);
if any(idx_t_infeasible)
  msg ='Ladder Plan: PLAN CONTAINS INFEASIBLE POINTS'; disp(msg); send_status(6,0,0,msg);
end
x_data = [q_data;zeros(size(q_data))];

% Plot COM traj
for i = 1:size(x_data,2)
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
