function [pose, INSCompensator] = Update_INS(pose__, INSCompensator)

pose = pose__;

% INSCompensator.utime
% INSCompensator.biases.bg'
% INSCompensator.biases.ba'
% INSCompensator.dE_l'
% INSCompensator.dV_l'
% INSCompensator.dP_l'
% pause

% Incorporate EKF based state misalignment
pose.lQb = qprod(pose__.lQb, e2q(INSCompensator.dE_l));
INSCompensator.dlQl = [1;0;0;0];

pose.V_l = pose__.V_l - INSCompensator.dV_l;
INSCompensator.dV_l = [0;0;0];
pose.P_l = pose__.P_l - INSCompensator.dP_l;
INSCompensator.dP_l = [0;0;0];


