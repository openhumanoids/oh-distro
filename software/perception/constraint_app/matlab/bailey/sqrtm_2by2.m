function X = sqrtm_2by2(A)
%SQRTM     Matrix square root.
%   X = SQRTM_2by2(A) is the principal square root of the matrix A, i.e. X*X = A.
%          
%   X is the unique square root for which every eigenvalue has nonnegative
%   real part.  If A has any eigenvalues with negative real parts then a
%   complex result is produced.  If A is singular then A may not have a
%   square root.  A warning is printed if exact singularity is detected.
%          
% Adapted for speed for 2x2 matrices from the MathWorks sqrtm.m implementation.
% Tim Bailey 2004.

[Q, T] = schur(A);        % T is real/complex according to A.
%[Q, T] = rsf2csf(Q, T);   % T is now complex Schur form.

R = zeros(2);

R(1,1) = sqrt(T(1,1));
R(2,2) = sqrt(T(2,2));
R(1,2) = T(1,2) / (R(1,1) + R(2,2));

X = Q*R*Q';
