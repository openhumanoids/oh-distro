function [x, dx] = subsetOfMarkersMeasuredMarkerFunction(params, marker_positions_measured)
x = marker_positions_measured;
measured_markers = ~any(isnan(marker_positions_measured), 1);
x(:, ~measured_markers) = reshape(params, 3, sum(~measured_markers));
dx = zeros(numel(x), numel(params));
dx = setSubMatrixGradient(dx, eye(numel(params)), 1:3, find(~measured_markers), size(x));
end