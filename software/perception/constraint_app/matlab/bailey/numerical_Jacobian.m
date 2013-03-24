function J = numerical_Jacobian(x, model, dmodel, offset, varargin)
% J = numerical_Jacobian(x, model, dmodel, offset, ...)
%
% INPUTS:
%   x - state
%   model - function handle for non-linear model: either ('myfunc') or (@myfunc).
%   dmodel - function handle for model residuals: e = dmodel(y1, y2)
%   offset - small increment to generate numerical tangent. If passed [], uses default value.
%   ... - optional arguments such that model has form: y = model(x, ...)
%
% OUTPUT:
%   J - Jacobian of y with respect x.
%
% NOTES:
%   1. This function computes an approximate Jacobian for a non-linear model.
%
%   2. The 'dmodel' is necessary to deal with discontinuous models, where the true residuals do
%   not match the difference in two values. A classic example is the difference between two normalised
%   polar values. If you have a continuous model, or a model with a trivial residual, you may simply
%   pass [] to 'dmodel'.
%
% Tim Bailey 2004.

if isempty(offset), offset = 1e-9; end
if isempty(dmodel), dmodel = @default_dmodel; end

y = feval(model, x, varargin{:});
lenx = length(x);
leny = length(y);

xt = x;
J = zeros(leny,lenx);

for i=1:lenx
    xt(i) = x(i) + offset;
    yt = feval(model, xt, varargin{:});
    dy = feval(dmodel, yt, y);
    J(:,i) = dy/offset;
    xt(i) = x(i);
end

%
%

function e = default_dmodel(x1, x2)
e = x1 - x2;
