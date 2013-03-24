function s = uniform_random(N)
%function s = uniform_random(N)
%
% Generate N uniform-random numbers within interval (0,1) in ascending order.
% Adapted from Doucet & de Feitas' high speed Niclas Bergman Procedure.
%
% Tim Bailey 2005.

s = fliplr(cumprod(rand(1,N).^(1./(N:-1:1))));

% Alternatively, a solution of my own devising (TB, 2006). But is it uniform-random?
%s = cumsum(rand(1,N));
%s = s ./ (s(end)+rand(1));
