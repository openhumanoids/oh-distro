function [num,den] = secondorderNotch_mfallon(Wo,BW)
% Design a 2nd-order notch digital filter.
%
% mfallon: simplified version of matlab toolbox function

Ab = abs(10*log10(.5));

% Inputs are normalized by pi.
BW = BW*pi;
Wo = Wo*pi;

Gb   = 10^(-Ab/20);
beta = (sqrt(1-Gb.^2)/Gb)*tan(BW/2);
gain = 1/(1+beta);

% 1x3 vectors
num  = gain*[1 -2*cos(Wo) 1];
den  = [1 -2*gain*cos(Wo) (2*gain-1)];