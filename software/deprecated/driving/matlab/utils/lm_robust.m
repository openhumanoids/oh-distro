function result = lm_robust(prob)

%TODO: warnings on singular matrix


if (~isfield(prob,'jacobian_func'))
    prob.jacobian_func = @numeric_jacobian;
end
if (~isfield(prob,'weight_model'))
    prob.weight_model = 'none';
end
if (~isfield(prob,'max_iterations'))
    prob.max_iterations = 100;
end
if (~isfield(prob,'min_error_change'))
    prob.min_error_change = 1e-6;
end
if (~isfield(prob,'min_parameter_change'))
    prob.min_parameter_change = 1e-6;
end
if (~isfield(prob,'debug'))
    prob.debug = true;
end

switch (prob.weight_model)
    case 'none'
        prob.robust_weight_func = @noop_weight;
        prob.robust_param_func = @noop_param;
        prob.robust_objective_func = @noop_objective;
    case 'tukey'
        prob.robust_weight_func = @tukey_weight;
        prob.robust_param_func= @tukey_param;
        prob.robust_objective_func = @tukey_objective;
    case 'custom'
        % user has to fill prob.weight_func and prob.weight_param_func
end

lambda_init = false;
x = prob.x;
e = prob.error_func(prob,x);

success = true;
for iter = 1:prob.max_iterations
    % Compute weights
    robust_param = prob.robust_param_func(prob,e);
    w = prob.robust_weight_func(e,robust_param);
    
    % Compute error from previous step
    prev_error = sum(prob.robust_objective_func(e,robust_param));
    
    % Diagnostics
    if (prob.debug)
        fprintf('iter %d %f %f\n', iter, prev_error, robust_param);
    end
    
    % Set up matrix equations
    jac = prob.jacobian_func(prob, x);
    jac = jac.*repmat(w(:),[1,size(jac,2)]);
    jac2 = jac'*jac;
    rhs = jac'*(w(:).*e(:));
    
    % Increase lambda until error decreases
    derror = 1e10;
    while (derror > 0)
        
        % initialize lambda
        if (~lambda_init)
            lambda = 1e-3 * mean(diag(jac2));
            lambda_init = true;
        end
        

        % Solve matrix equations
        lhs = jac2 + lambda*eye(numel(x));
        delta = lhs\rhs;

        % Determine error
        x_test = x - delta;
        e_test = prob.error_func(prob, x_test);
        curr_error = sum(prob.robust_objective_func(e_test,robust_param));
        
        derror = curr_error - prev_error;
        if (derror > 0)
            lambda = lambda*10;
        end
    end
    
    % When error decreases, lower lambda and check stopping criteria
    lambda = lambda / 10;
    x = x_test;
    e = e_test;
    if (-derror < prev_error*prob.min_error_change)
        break;
    end

    if (all(abs(delta)<x*prob.min_parameter_change))
        break;
    end
end

% fill results
result.x = x;
result.errors = e;
result.weights = w;
result.status = success;
result.iters = iter;
result.cov = pinv(jac'*jac);



%% non-analytic (numerically approximated) derivatives of the error
function J = numeric_jacobian(prob,x)

eminus = prob.error_func(prob,x);
J = zeros(numel(eminus),numel(x));

for i = 1:size(J,2)
    small_value = max(x(i)*1e-4, 1e-6);
    xplus = x;
    xplus(i) = xplus(i)+small_value;
    eplus = prob.error_func(prob,xplus);
    J(:,i) = (eplus(:)-eminus(:))/small_value;
end



%% no robust weight
function w = noop_weight(e,params)
w = ones(size(e));

function params = noop_param(prob,e)
params = [];

function e = noop_objective(e,params)
e = e.^2;


%% tukey biweight
function w = tukey_weight(e,params)
e2 = e.^2;
w = (1-e2/params(1));
w(e2>params(1)) = 0;

function params = tukey_param(prob,e)
params = 1.48*sqrt(median(e.^2))*4.6851;
params = max(params^2,prob.min_tukey_param^2);

function e = tukey_objective(e,params)
w = tukey_weight(e,params);
e = 1-w.*w.*w;
