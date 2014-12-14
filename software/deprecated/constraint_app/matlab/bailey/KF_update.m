function [x,P]= KF_update(x,P,v,R,H)
%function [x,P] = KF_update(x,P,v,R,H)
%
% Calculate the Kalman filter update given the prior state [x,P], the innovation v, the 
% observe uncertainty R, and the linear observation model H. 
%
% Tim Bailey 2005.

[x,P]= KF_update_cholesky(x,P,v,R,H);
