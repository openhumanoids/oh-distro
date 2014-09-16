function marker_data = measuredViconMarkerPositions()

%NOTEST
% standoff and marker measurements
standoff_length = 31e-3;
standoff_offset = 1.74e-3; % offset due to fact that standoffs aren't flush against surface:
ball_diameter = 15e-3;

% Right hand:
r_reference = [-0.068628; -0.093738; -0.020245]; % obtained from meshlab
r_reference_to_base = [0.017; 0.015; 0.0]; % measured offset of base w.r.t. reference
r_base = r_reference + r_reference_to_base;
r_base_to_marker = [0; 0; -(standoff_offset + standoff_length + ball_diameter / 2)];
r_marker = r_base + r_base_to_marker;
% marker was measured in visual frame. The visual is rotated by pi about the
% y-axis w.r.t. the body frame in the urdf, so need to rotate back to body
% frame
R_visual_to_body = rpy2rotmat([0; pi; 0]);
r_marker = R_visual_to_body * r_marker;
marker_data.r_hand.num_markers = 5;
r_markers = nan(3, marker_data.r_hand.num_markers);
r_markers(:, 4) = r_marker;
marker_data.r_hand.num_params = sum(sum(isnan(r_markers)));
marker_data.r_hand.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, r_markers);

% Left hand:
% TODO: need to update these after urdf and mesh files have been fixed
l_reference = [0.0686277; 0.0937385; -0.0201257]; % obtained from meshlab
l_reference_to_base = [-0.017; -0.015; 0.0]; % measured offset of base w.r.t. reference
l_base = l_reference + l_reference_to_base;
l_base_to_marker = [0; 0; -(standoff_offset + standoff_length + ball_diameter / 2)];
l_marker = l_base + l_base_to_marker;
marker_data.l_hand.num_markers = 5;
l_markers = nan(3, marker_data.l_hand.num_markers);
l_markers(:, 3) = l_marker;
marker_data.l_hand.num_params = sum(sum(isnan(l_markers)));
marker_data.l_hand.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, l_markers);

% Torso:
torso_reference = [0.215498; -0.076292; 0.295373]; % obtained from meshlab
torso_reference_to_marker = [10e-3; 30e-3; 0];
torso_marker = torso_reference + torso_reference_to_marker;
marker_data.utorso.num_markers = 3;
torso_markers = nan(3, marker_data.utorso.num_markers);
torso_markers(:, 2) = torso_marker;
torso_markers(1, :) = torso_marker(1); % same x
marker_data.utorso.num_params = sum(sum(isnan(torso_markers)));
marker_data.utorso.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, torso_markers);

end