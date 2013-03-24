function [x,P]= KF_update_cholesky(x,P,v,R,H)
%function [x,P]= KF_update_cholesky(x,P,v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P], the innovation v, the 
% observe uncertainty R, and the (linearised) observation model H. The result is calculated 
% using Cholesky factorisation, which is more numerically stable than a naive implementation.
%
% Adapted from code by Jose Guivant.
%
% Tim Bailey 2003.

PHt = P*H';
S = H*PHt + R;

%S = (S+S')*0.5; % ensure S is symmetric 
Sc  = chol(S);  % note: S = Sc'*Sc
Sci = inv(Sc);  % note: inv(S) = Sci*Sci'

Wc = PHt * Sci;
W  = Wc * Sci';

x = x + W*v; % update 
P = P - Wc*Wc';
