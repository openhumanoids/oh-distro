function x = chi_square_bound(prob, n)
%function x = chi_square_bound(prob, n)
%
% INPUTS: 
%   prob - probability [0, 1)
%   n - degrees of freedom
%
% OUTPUT: 
%   x - point on the x-axis where the Chi-squared probability mass F(x) equals prob.
%
% Tim Bailey 2005.

if prob < 0 | prob >= 1, error('Probability must be in interval [0, 1)'), end

% Hunt for upper bound
upper = 5;
while 1
    f = chi_square_mass(upper, n);
    if f >= prob, break, end
    upper = upper * 2;
end

% Perform bounded search for root
x = fzero(@root_fnctn, [0; upper], optimset('disp','off'), prob, n);

%
%

function y = root_fnctn(x, prob, n)
y = chi_square_mass(x, n) - prob;
