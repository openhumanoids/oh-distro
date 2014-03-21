function [ RESULTS ] = setupUnitTest1Results( iterations )
%SETUPUNITTEST1RESULTS prepares array based memory for comparing true and
%estimated poses for the first motion simulator unit test

% Residuals from the control test INS in MATLABland
RESULTS.trueINSPoseResiduals.P_l = zeros(iterations,3);
RESULTS.trueINSPoseResiduals.V_l = zeros(iterations,3);
RESULTS.trueINSPoseResiduals.f_l = zeros(iterations,3);

RESULTS.trueINSPoseResiduals.q = zeros(iterations,4);

% These are the residuals from the state-estimate process
RESULTS.cppINSPoseResiduals.P_l = zeros(iterations,3);
RESULTS.cppINSPoseResiduals.V_l = zeros(iterations,3);
RESULTS.cppINSPoseResiduals.f_l = zeros(iterations,3);

RESULTS.cppINSPoseResiduals.q = zeros(iterations,4);

end

