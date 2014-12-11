function idx = index_table(X, ielem)
%function idx = index_table(X, ielem)
%
% INPUTS:
%   X - an (M x N x P x ...) table of values
%   ielem - a matrix of index vectors (Ndimensions x Nindices)
%
% OUTPUTS:
%   idx - a (1 x Nindices) array of indices into X(:)
%
% Given an hyper-dimensional matrix X, and a set of indices of
% particular elements of the matrix, this function returns the 
% equivalent indices for those elements when X is treated as
% a column vector. For example,
%
%   X = [1 2 3; 
%        4 5 6];
%
%   ielem = [2 1;   % that is, indices (2,2) and (1,3)
%            2 3];
%
%   idx = index_table(X, ielem); 
%                   % idx == [4 5]
%                   % such that, X(4) == 5 and X(5) == 3
%
% That is, X(2,2) == X(4) and X(1,3) == X(5). We may then write 
% element manipulation operations, such as:
%
%   X(idx) = X(idx) + 7;    % now, X == [1 2 10; 4 12 6]
%   
% Tim Bailey 2005.
%
% Note (TB, 2006) ------------------------------------------
% I have since found that:
%   idx = index_table(X, ielem); is equivalent to calling 
%   idx = sub2ind(size(X), ielem(1,:), ielem(2,:), ...);
% where sub2ind is a Matlab function.
% ----------------------------------------------------------

dims = size(X);
mult = dims(1);

idx = ielem(1,:);
for i=2:length(dims)
    idx = idx + mult*(ielem(i,:)-1);
    mult = mult*dims(i);
end

% example index calculations, 
% 2D: idx = i + M*(j-1)
% 3D: idx = i + M*(j-1) + M*N*(k-1)
