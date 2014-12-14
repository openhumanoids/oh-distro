function K = distance_KLD(v, P, Q)
%function K = distance_KLD(v, P, Q)
%
% INPUTS:
%   v - difference between two means v = p - q
%   P, Q - covariance matrices
%
% OUTPUT:
%   K - Kullback Leibler divergence (relative entropy) between two Gaussians.
%
% References:
%   Jacob Goldberger and Sam Roweis, Hierarchical Clustering of a Mixture Model, 
%   Neural Information Processing Systems, 2004.
%   http://www.cs.toronto.edu/~roweis/publications.html
%   
%   Shaohua Kevin Zhou and Rama Chellappa, From sample similarity to ensemble 
%   similarity: Probabilistic distance measures in reproducing kernel Hilbert 
%   space. IEEE Transactions on Pattern Analysis and Machine Intelligence (to 
%   appear). http://www.umiacs.umd.edu/~shaohua/publications.html
%
% Tim Bailey 2005.

Qi = inv(Q);
D = size(P,1);

K1 = v'*Qi*v;
K2 = log(det(Q)/det(P));
K3 = trace(Qi*P);

K = 0.5*(K1 + K2 + K3 - D);
