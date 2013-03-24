function R = sqrt_posdef(P, type)
%function R = sqrt_posdef(P, type)
%
% INPUTS: 
%   P - symmetric positive definite matrix
%   type - type of square-root operation to perform
%       1. svd (default)
%       2. chol
%       3. sqrtm
%       4. eig
%
% OUTPUT:
%   R - square-root of P, such that P = R*R'. (In case of sqrtm, P = R*R.)
%
% Compute the square-root of a symmetric positive definite matrix
%
% Tim Bailey, 2006.

if nargin == 1, type = 1; end

switch type
    case 1 % svd decomposition, P = U*D*U' = R*R' (UDU form is also called modified Cholesky decomposition)
        [U,D,V] = svd(P);
        R = U*sqrt(D);
        %R = reprow(sqrt(diag(D))', size(U,1)) .* U;
    case 2 % cholesky decomposition (triangular), P = R*R'
        R = chol(P)';
    case 3 % principal square root (symmetric), P = R*R
        R = sqrtm(P);
    case 4 % eigenvectors, P*V = V*D
        [V,D] = eig(P);
        R = V*sqrt(D);
    otherwise
        error('Invalid type selection')
end
