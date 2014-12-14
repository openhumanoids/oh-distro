function J = numerical_Jacobian_cd(x, model, dmodel, offset, varargin)
% J = numerical_Jacobian_cd(x, model, dmodel, offset, ...)
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
%   This function performs the same basic operation as does numerical_Jacobian.m; 
%   it computes a Jacobian matrix by finite differencing. However, where the other 
%   function uses the "forward-difference" formula, this function uses the
%   "central difference" formula, which is more accurate but twice as expensive.
%   The 
%
% Tim Bailey 2007.

if isempty(offset), offset = 1e-9; end
if isempty(dmodel), dmodel = @default_dmodel; end

y = feval(model, x, varargin{:});
lenx = length(x);
leny = length(y);

xu = x;
xl = x;
off2 = offset*2;
J = zeros(leny,lenx);

for i=1:lenx
    xu(i) = x(i) + offset;
    xl(i) = x(i) - offset;    
    
    yu = feval(model, xu, varargin{:});
    yl = feval(model, xl, varargin{:});
    dy = feval(dmodel, yu, yl);
    J(:,i) = dy/off2;
    
    xu(i) = x(i);
    xl(i) = x(i);
end

%
%

function e = default_dmodel(x1, x2)
e = x1 - x2;
