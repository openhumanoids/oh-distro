function [y, Y] = unscented_transform(func, dfunc, x, P, varargin)
%function [y, Y] = unscented_transform(func, dfunc, x, P, ...)
%
% Algorithm implemented as described in: 
%   A New Method for the Nonlinear Transformation of Means and Covariances in Filters and Estimators
%   Simon Julier, Jeffrey Uhlmann, and Hugh F. Durrant-Whyte, pp 477--482
%   IEEE TRANSACTIONS ON AUTOMATIC CONTROL, VOL. 45, NO. 3, MARCH 2000
%
% INPUTS:
%   func - function handle (@myfunc) or string ('myfunc') for non-linear transform.
%   dfunc - function handle (or string) for residual between two transformed values: e = mydfunc(y1, y2).
%   x, P - initial mean and covariance.
%   ... - optional arguments such that 'func' has form: y = myfunc(x, ...).
%
% OUTPUTS:
%   y, Y - transformed mean and covariance.
%
% NOTES:
%   1. The unscented filter has two primary advantages over the EKF: (i) it produces a more accurate
%   estimate of the first and second moments of a random vector transformed through a non-linear 
%   function, and (ii) it does not require analytical Jacobians.
%
%   2. The function 'func' is the non-linear function itself and transforms 'x' to 'y'. This 
%   function may be passed any number of additional parameters.
%       eg, y = myfunc(x, p1, p2, p3);
%
%   3. The function 'dfunc' is required to deal with discontinuous functions. Some non-linear 
%   functions are discontinuous, but their residuals are not equal to the discontinuity. A classic
%   example is a normalised polar measurement:
%       y1 = myfunc(x1, p1, p2, p3);    % lets say y1 == pi
%       y2 = myfunc(x2, p1, p2, p3);    % lets say y2 == -pi
%       dy = y1 - y2;                   % dy == 2*pi -- this is wrong (must be within +/- pi)
%       dy = mydfunc(y1, y2);           % dy == 0 -- this is correct
%   Thus, 'mydfunc' is a function that computes the true residual of y1-y2. If the function 'myfunc'
%   is not discontinuous, or has a trivial residual, just pass [] to parameter 'dfunc'.
%
%   4. The functions 'func' and 'dfunc' must be vectorised. That is, they must be able to accept a set of 
%   states as input and return a corresponding set of results. So, for 'func', the state x will not be a 
%   single column vector, but a matrix of N column vectors. Similarly, for 'dfunc', the parameters y1 and 
%   y2 will each be matrices of N column vectors.
%
% EXAMPLE USE:
%   [y,Y] = unscented_transform(@myfunc, @mydfunc, x,P, p1, p2, p3);
%   [a,B] = unscented_transform('continuous_model', [], x,P);
%
% Tim Bailey 2003, modified 2004, 2005.

% Set up some values
D = length(x);  % state dimension
N = D*2 + 1;    % number of samples
scale = 3;      % want scale = D+kappa == 3
kappa = scale-D;

% Create samples
%Ps = chol(P)' * sqrt(scale); 
Ps = sqrt_posdef(P) * sqrt(scale); 
ss = [x, repvec(x,D)+Ps, repvec(x,D)-Ps];

% Transform samples according to function 'func'
if isempty(dfunc), dfunc = @default_dfunc; end
ys = feval(func, ss, varargin{:});  % compute (possibly discontinuous) transform
base = repvec(ys(:,1), N);          % set first transformed sample as the base
delta = feval(dfunc, base, ys);     % compute correct residual
ys = base - delta;                  % offset ys from base according to correct residual

% Calculate predicted observation mean
idx = 2:N;
y = (2*kappa*ys(:,1) + sum(ys(:,idx),2)) / (2*scale); 

% Calculate new unscented covariance
dy = ys - repvec(y,N);
Y  = (2*kappa*dy(:,1)*dy(:,1)' + dy(:,idx)*dy(:,idx)') / (2*scale);
% Note: if x is a matrix of column vectors, then x*x' produces the sum of outer-products.

%
%

function e = default_dfunc(y1, y2)
e = y1 - y2;

%
%

function x = repvec(x,N)
x = x(:, ones(1,N));
