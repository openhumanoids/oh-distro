function [pose, INSCompensator] = Update_INS(pose__, INSCompensator)

pose = pose__;

% INSCompensator.utime
% INSCompensator.biases.bg
% INSCompensator.biases.ba
% INSCompensator.dE_l
% INSCompensator.dV_l
% INSCompensator.dP_l


% Incorporate EKF estimated error updates
pose.lQb = qprod(pose__.lQb, e2q(INSCompensator.dE_l));
pose.V_l = pose__.V_l - INSCompensator.dV_l;
pose.P_l = pose__.P_l - INSCompensator.dP_l;

% Reset information container, since it has now been added to the INS state
% INSCompensator.dlQl = [1;0;0;0];
INSCompensator.dE_l = [0;0;0];
INSCompensator.dV_l = [0;0;0];
INSCompensator.dP_l = [0;0;0];

% pause

