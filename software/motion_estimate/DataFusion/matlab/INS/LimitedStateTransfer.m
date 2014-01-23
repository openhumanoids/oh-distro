function [ x, INSCompensator ] = LimitedStateTransfer( uts, x, limitedFB, INSCompensator )
% 15 state limited feeedback step

% we move misalignment information out of the filter to achieve better
% linearization
INSCompensator.utime = uts;
INSCompensator.dE_l = - limitedFB*x(1:3);
INSCompensator.dV_l = - limitedFB*x(7:9);
INSCompensator.dP_l = - limitedFB*x(13:15);
INSCompensator.biases.bg = INSCompensator.biases.bg + limitedFB*x(4:6);
INSCompensator.biases.ba = INSCompensator.biases.ba + limitedFB*x(10:12);

x = (1-limitedFB)*x;

end

