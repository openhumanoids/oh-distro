function [x,P]= EKF_update(zfunc, dzfunc, x,P, z,R, varargin)
%[x,P] = EKF_update(zfunc,dzfunc, x,P, z,R, ...)
%
% This function performs an extended Kalman filter update with numerical approximation of 
% the linearised observation Jacobian H. The resulting EKF-update operation is as simple 
% to use as unscented_update.m.
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
%   1. This function performs an extended Kalman filter update step and numerically approximates H.
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
% EXAMPLE USE:
%   [x,P] = EKF_update(@angle_observe_model, @residual_model, x,P, z,R, a, b, c);
%   [x,P] = EKF_update('my_simple_model', [], x,P, z,R);
%
% Tim Bailey 2005.

if isempty(dzfunc), dzfunc = @default_dfunc; end

zpred = feval(zfunc, x, varargin{:});
v = feval(dzfunc, z, zpred);
H = numerical_Jacobian(x, zfunc, dzfunc, [], varargin{:});
[x,P] = KF_update(x,P, v,R,H);   

%
%

function e = default_dfunc(y1, y2)
e = y1 - y2;
