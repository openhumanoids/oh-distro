function [x,P,ss,zs] = unscented_update(zfunc, dzfunc, x,P, z,R, varargin)
%[x,P] = unscented_update(zfunc,dzfunc, x,P, z,R, ...)
%
% Algorithm implemented as described in: 
%   Simon Julier's PhD thesis pp 20--23.
%
% INPUTS:
%   zfunc - function handle (@myfunc) or string ('myfunc') for observe model.
%   dzfunc - function handle (or string) for observation residual: v = mydfunc(z, z_predict);
%   x, P - predict mean and covariance
%   z, R - observation and covariance (observation noise is assumed additive)
%   ... - optional arguments such that 'zfunc' has the form: z = myfunc(x, ...)
%
% OUTPUTS:
%   x, P - updated mean and covariance
%
% NOTES:
%   1. This function uses the unscented transform to compute a Kalman update.
%
%   2. The function 'zfunc' is the non-linear observation function. This function may be passed 
%   any number of additional parameters.
%       eg, z = my_observe_model(x, p1, p2);
%
%   3. The residual function 'dzfunc' is required to deal with discontinuous functions. Some non-linear 
%   functions are discontinuous, but their residuals are not equal to the discontinuity. A classic
%   example is a normalised angle measurement model:
%       z1 = angle_observe_model(x1, p1, p2, p3);   % lets say z1 == pi
%       z2 = angle_observe_model(x2, p1, p2, p3);   % lets say z2 == -pi
%       dz = z1 - z2;                               % dz == 2*pi -- this is wrong (must be within +/- pi)
%       dz = residual_model(z1, z2);                % dz == 0 -- this is correct
%   Thus, 'residual_model' is a function that computes the true residual of z1-z2. If the function 
%   'zfunc' is not discontinuous, or has a trivial residual, just pass [] to parameter 'dzfunc'.
%
%   4. The functions 'zfunc' and 'dzfunc' must be vectorised. That is, they must be able to accept a set of 
%   states as input and return a corresponding set of results. So, for 'zfunc', the state x will not be a 
%   single column vector, but a matrix of N column vectors. Similarly, for 'dzfunc', the parameters z and 
%   z_predict will be equal-sized matrices of N column vectors.
%
% EXAMPLE USE:
%   [x,P] = unscented_update(@angle_observe_model, @residual_model, x,P, z,R, a, b, c);
%   [x,P] = unscented_update('range_model', [], x,P, z,R);
%
% Tim Bailey 2003, modified 2004, 2005.

% Set up some values
D = length(x);  % state dimension
N = D*2 + 1;    % number of samples
scale = 1;      % want scale = D+kappa == 3
kappa = scale-D;

% Create samples
%Ps = chol(P)' * sqrt(scale); 
Ps = sqrt_posdef(P) * sqrt(scale); 
ss = [x, repvec(x,D)+Ps, repvec(x,D)-Ps];

% Transform samples according to function 'zfunc' to obtain the predicted observation samples
if isempty(dzfunc), dzfunc = @default_dfunc; end
zs = feval(zfunc, ss, varargin{:}); % compute (possibly discontinuous) transform
if any(isnan(zs)), return; end
zz = repvec(z,N);
dz = feval(dzfunc, zz, zs); % compute correct residual
zs = zz - dz;               % offset zs from z according to correct residual

% Calculate predicted observation mean
zm = (kappa*zs(:,1) + 0.5*sum(zs(:,2:end), 2)) / scale; 

% Calculate observation covariance and the state-observation correlation matrix
dx = ss - repvec(x,N);
dz = zs - repvec(zm,N);
Pxz = (2*kappa*dx(:,1)*dz(:,1)' + dx(:,2:end)*dz(:,2:end)') / (2*scale);
Pzz = (2*kappa*dz(:,1)*dz(:,1)' + dz(:,2:end)*dz(:,2:end)') / (2*scale);

Pzz = force_spd(Pzz);

% Compute Kalman gain
S = Pzz + R;
Sc  = chol(S);  % note: S = Sc'*Sc
Sci = inv(Sc);  % note: inv(S) = Sci*Sci'
Wc = Pxz * Sci;
W  = Wc * Sci';

% Perform update
% x = x + W*(z - zm);
x = x + W*dzfunc(z', zm')';
P = P - Wc*Wc';
P = force_spd(P);
% PSD_check = chol(P);

%
%
function S2 = force_spd(S1)
    % force the matrix to be SPD
    [u,s,v]=svd(S1);
    L = u*sqrt(s);
    S2 = L*L';



%
%

function e = default_dfunc(y1, y2)
e = y1 - y2;
