function Pinv = inv_posdef(P)
%function Pinv = inv_posdef(P)
%
% Invert a positive definite matrix P. More numerically stable than Pinv = inv(P)
% and result is guaranteed to remain symmetric.
% Adapted from INV_POSDEF.m of the Lightspeed library by Minka.
%
% Tim Bailey 2005.

Pc = chol(P);
Pci = inv(Pc);
Pinv = Pci*Pci';
