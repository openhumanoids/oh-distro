function [eq, violation_id] = violations(state)    
    % assume model has no joint constraints
    eq = zeros(size(state,1),1);
    violation_id = zeros(size(state,1),1);