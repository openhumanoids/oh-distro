function [x,P] = KF_update_IEKF(x,P, z,R, hfun,hjac, N, varargin)
%function [x,P] = KF_update_IEKF(x,P, z,R, hfun,hjac, N, ...)
%
% INPUTS:
%   x - x(k|k-1) - predicted state
%   P - P(k|k-1) - predicted covariance
%   z - observation
%   R - observation uncertainty
%   hfun - function for computing the innovation: v = hfun(x,z,...);
%   hjac - function for computing the observation model jacobian: H = hjac(x,...);
%   N - number of iterations of the IEKF update
%   ... - optional parameters to pass to hfun and hjac
%
% OUTPUTS:
%   x - x(k|k) - a posteri state
%   P - P(k|k) - a posteri covariance
%
% Uses iterated EKF (cite Bar-Shalom, 2001, p406).
%
% Tim Bailey 2004, modified 2005.

x0 = x; % prior values
P0 = P;
P0i = inv_posdef(P);
Ri  = inv_posdef(R);

for i=1:N % iterate solution
    H = feval(hjac,x,varargin{:});
    P = calculate_P(P0,H,R);
    
    v = feval(hfun,x,z,varargin{:}); % to cope with discontinuous models, want this form rather than: v = z - feval(hfun,x); 
    x = calculate_x(v,x,P, x0,P0i, H,Ri);    
end

H = feval(hjac,x,varargin{:}); % final iteration 
P = calculate_P(P0,H,R);

%
%

function P = calculate_P(P,H,R)
PHt = P*H';
S = H*PHt + R;
Sci = inv(chol(S));
Wc = PHt * Sci;
P = P - Wc*Wc';

function x = calculate_x(v,x,P,x0,P0i,H,Ri)
M1 = P * H' * Ri; 
M2 = P * P0i * (x-x0);
x = x + M1*v - M2;

function Ai = inv_posdef(A)
Ac = chol(A);
Aci = inv(Ac);
Ai = Aci*Aci';
