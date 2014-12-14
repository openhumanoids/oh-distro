function [x,P]= KF_update_simple(x,P,v,R,H)
%function [x,P]= KF_update_simple(x,P,v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P],
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using a naive inversion of S, and is
% less numerically stable than the Cholesky factorisation based 
% update (see KF_update_cholesky).
%
% Tim Bailey 2003

PHt= P*H';
S= H*PHt + R;
Si= inv(S);
Si= make_symmetric(Si);
PSD_check= chol(Si);
W= PHt*Si;

x= x + W*v; 
P= P - make_symmetric(W*S*W');
PSD_check= chol(P);

function P= make_symmetric(P)
P= (P+P')*0.5;
