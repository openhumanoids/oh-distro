function [support_times, supports, comtraj, foottraj, V, zmptraj,c] = walkingPlanFromSteps(biped, x0, footsteps)

nq = getNumDOF(biped);
q0 = x0(1:nq);
kinsol = doKinematics(biped,q0);

options.full_foot_pose_constraint = true;
[zmptraj,foottraj, support_times, supports] = planZMPTraj(biped, q0, footsteps,options);
zmptraj = setOutputFrame(zmptraj,desiredZMP);
%% construct ZMP feedback controller
com = getCOM(biped,kinsol);
foot_pos = contactPositions(biped,kinsol,false,struct('terrain_only',true,'body_idx',[biped.foot_bodies_idx.right, biped.foot_bodies_idx.left]));
zfeet = mean(foot_pos(3,:));

% get COM traj from desired ZMP traj
options.com0 = com(1:2);
[c,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(com(3)-zfeet,zmptraj,options);
