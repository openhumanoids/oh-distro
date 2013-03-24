function [Bc,Bd] = distance_bhattacharyya(v, P, Q)
%[Bc,Bd] = distance_bhattacharyya(v, P, Q)
%
% INPUTS:
%   v - difference between two means v = p - q
%   P, Q - covariance matrices
%
% OUTPUTS:
%   Bc - Bhattacharyya coefficient
%   Bd - Bhattacharyya distance
%
% REFERENCES:
%   Shaohua Kevin Zhou and Rama Chellappa, From sample similarity to ensemble 
%   similarity: Probabilistic distance measures in reproducing kernel Hilbert 
%   space. IEEE Transactions on Pattern Analysis and Machine Intelligence (to 
%   appear). http://www.cfar.umd.edu/~shaohua/publications.html
%
% Bhattacharyya distance between two Gaussians.
%
% Tim Bailey 2005.

S = 0.5*(P + Q);
Bd = 0.125*v'*inv(S)*v + 0.5*log(det(S)/sqrt(det(P)*det(Q)));
Bc = exp(-Bd);
