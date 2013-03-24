function B = inv_pseudo(A)
%function B = inv_pseudo(A)
%
% The Moore-Penrose pseudo-inverse (or generalised inverse). 
%
% Computes B such that B*A = I. (Note, A*B will not equal I, in general, 
% unless A is invertible.) Commonly used to solve to over-constrained
% systems of the form, A*x = c, in the least-squares sense, x = B*c.
%
% Tim Bailey 2007.

B = (A'*A)\A'; % or B = inv(A'*A)*A';
