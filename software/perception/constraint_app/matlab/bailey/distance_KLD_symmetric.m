function K = distance_KLD_symmetric(v, P, Q)
%function K = distance_KLD_symmetric(v, P, Q)
%
% INPUTS:
%   v - difference between two means v = p - q
%   P, Q - covariance matrices
%
% OUTPUT:
%   K - Symmetric Kullback Leibler divergence between two Gaussians.
%
% References:
%   Shaohua Kevin Zhou and Rama Chellappa, From sample similarity to ensemble 
%   similarity: Probabilistic distance measures in reproducing kernel Hilbert 
%   space. IEEE Transactions on Pattern Analysis and Machine Intelligence (to 
%   appear). http://www.cfar.umd.edu/~shaohua/publications.html
%
% Tim Bailey 2005.


Pi = inv(P);
Qi = inv(Q);

K1 = v'*(Pi+Qi)*v;
K2 = trace(Pi*Q + Qi*P - 2*eye(size(P)));

K = 0.5*(K1 + K2);
