function s = stratified_random(N)
%function s = stratified_random(N)
%
% Generate N uniform-random numbers stratified within interval (0,1).
% The set of samples, s, are in ascending order.
%
% Tim Bailey 2003, modified 2005.

k = 1/N;
di = 0:k:(1-k); % deterministic intervals
s = di + rand(1,N) * k; % dither within interval
