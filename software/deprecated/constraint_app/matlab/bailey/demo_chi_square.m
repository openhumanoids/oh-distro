function demo_chi_square()
%function demo_chi_square
%
% Demonstrate simple use of the chi-square probability density function, 
% probability mass function, and probability bound.
%
% Tim Bailey 2005.

x = 0:0.1:12;   % x-axis domain
prob = 0.95;    % 95% bound
D = 3;          % dimension

f = chi_square_density(x, D);
F = chi_square_mass(x, D);
xb = chi_square_bound(prob, D);

figure
subplot(2,1,1)
plot(x,f, [xb xb], [0 0.4])
title(['Density function of degree ', num2str(D), '. Bound indicates ', num2str(prob*100),'% of area is to the left.'])

subplot(2,1,2)
plot(x,F, [xb xb], [0 1])
title(['Mass function of degree ', num2str(D), '. Bound indicates where y-axis = ', num2str(prob),'.'])
