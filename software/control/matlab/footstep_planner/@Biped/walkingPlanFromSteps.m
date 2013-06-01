function [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(biped, x0, footsteps, footstep_opts)

nq = getNumDOF(biped);
q0 = x0(1:nq);
kinsol = doKinematics(biped,q0);

[zmptraj,foottraj, support_times, supports] = planInitialZMPTraj(biped, q0, footsteps, footstep_opts);
zmptraj = setOutputFrame(zmptraj,desiredZMP);
%% construct ZMP feedback controller
com = getCOM(biped,kinsol);
zmap = getTerrainHeight(biped,com(1:2));
limp = LinearInvertedPendulum(com(3)-zmap,9.81,struct('ignore_frames',true));

% get COM traj from desired ZMP traj
options.use_tvlqr = false;
options.ignore_frames = true;
options.com0 = com(1:2);
[c,V,comtraj] = ZMPtracker(limp,zmptraj,options);
