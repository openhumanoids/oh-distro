function f=chi_square_mass(x,n)
%function F = chi_square_mass(x,n)
% INPUTS: x - x-axis coordinates, n - degrees of freedom
% OUTPUT: F(x) - Chi-square probability mass function
%
% Reference: Press, "Numerical Recipes in C", 2nd Ed., 1992, page 221.
%
% Tim Bailey 2005.

if any(x<0), error('x must be non-negative.'), end
f = gammainc(x/2, n/2) ;
