function [x,P,a] = covariance_intersection(x1,P1, x2,P2, a)
%function [x,P,a] = covariance_intersection(x1,P1, x2,P2, a)
%
% Inputs:
%   x1,x2 - mean vectors 
%   P1,P2 - covariance matrices
%   a - <optional> CI mixing factor
%
% Outputs:
%   x - fused mean
%   P - fused covariance
%   a - CI mixing factor used
%
% Notes:
%   Performs the covariance intersection algorithm for data fusion when
%   correlations (P12) between x1 and x2 are unknown. If the optional
%   argument, a, is not provided, it is computed to minimise the
%   determinant of P.
%
% Reference:
%   S.J. Julier and J.K. Uhlmann, A Non-divergent Estimation Algorithm in
%   the Presence of Unknown Correlations, in Proceedings of the American
%   Control Conference, June 1997.

P1i = inv_posdef(P1);
P2i = inv_posdef(P2);

if nargin == 4
    a = fminbnd(@det_ci, 0, 1, [], P1i, P2i);
end

P = inv_posdef(a*P1i + (1-a)*P2i);
x = P*(a*P1i*x1 + (1-a)*P2i*x2);

%
%

function d = det_ci(a, P1i, P2i)
Ri = a*P1i + (1-a)*P2i;
d = 1 / det(Ri); % det(R) == 1/det(inv(R))
