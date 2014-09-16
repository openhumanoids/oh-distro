function [x, dx] = subsetOfMarkersMeasuredMarkerFunction(params, marker_positions_measured)
x = marker_positions_measured;
variable_indices = find(isnan(marker_positions_measured));
x(variable_indices) = params;
dx = zeros(numel(x), numel(params));
for i = 1 : length(variable_indices)
  [row, col] = ind2sub(size(x), variable_indices(i));
  dx = setSubMatrixGradient(dx, 1, row, col, size(x), i);
end
end