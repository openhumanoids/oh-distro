function [x,P,w] = kmeans(s, x, N)
% function [x,P,w] = kmeans(samples, xinit, N)
%
% INPUTS: 
%   samples - set of samples
%   xinit - set of K initial cluster centres
%   N - number of iterations of kmeans
%
% OUTPUT:
%   x - final cluster centers
%   P - cluster covariances
%   w - cluster weights
%
% Performs the k-means clustering algorithm to find the set of K centres that
% "best" represent a set of samples according to a Euclidean cost function with
% hard data-association.
%
% Tim Bailey 2005.

[D, NS] = size(s);  % state dimension and number of samples
K = size(x,2);      % number of centres

P = zeros(D,D,K);
w = zeros(1,K);

while N > 0
    N = N-1;

    % Compute distances of each sample to each centre 
    d2 = (ones(NS,1)*sum(x.^2, 1))' + ... % this equation adapted from Netlab dist2.m
          ones(K,1)*sum(s.^2, 1) - ...
          2.*(x'*(s)); 

    % find nearest centre for each sample
    [dummy, ii] = min(d2,[],1);
    
    % compute new mean for each cluster
    for i = 1:K   
        sc = s(:, ii==i); 
        if ~isempty(sc)
            [x(:,i), P(:,:,i)] = sample_mean(sc);
            w(i) = size(sc,2) / NS;
        end
    end    
end
