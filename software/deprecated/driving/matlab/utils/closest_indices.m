function idx = closest_indices(t_actual, t_desired)

% assumes times are in ascending order
idx = zeros(numel(t_desired),1);
in_ptr = 1;

for i = 1:numel(t_desired)
    % Increment input time until it exceeds output time
    while (in_ptr < numel(t_actual) && t_actual(in_ptr) <= t_desired(i))
        in_ptr = in_ptr+1;
    end
    
    idx(i) = in_ptr-1;
end
