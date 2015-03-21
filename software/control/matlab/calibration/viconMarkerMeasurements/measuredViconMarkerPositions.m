function marker_data = measuredViconMarkerPositions()

%NOTEST
% standoff and marker measurements
% standoff_length = 31e-3;
% standoff_offset = 1.74e-3; % offset due to fact that standoffs aren't flush against surface:
ball_diameter = 15e-3;
plate_thickness = 2.5e-3;
marker_to_force_torque_sensor = [0; 5.4e-3 + 5.75e-3; 0];

% Right hand:
marker_data.r_hand.num_markers = 5;
r_markers = nan(3, marker_data.r_hand.num_markers);
r_hand_force_torque_sensor_to_link = [0; -0.1245; -0.0112];
r_markers(:, 1) = -marker_to_force_torque_sensor + r_hand_force_torque_sensor_to_link; % the minus is on purpose
marker_data.r_hand.num_params = sum(sum(isnan(r_markers)));
marker_data.r_hand.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, r_markers);

% Left hand:
marker_data.l_hand.num_markers = 5;
l_markers = nan(3, marker_data.l_hand.num_markers);
l_hand_force_torque_sensor_to_link = [0; -0.1245; 0.0112];
l_markers(:, 3) = -marker_to_force_torque_sensor + l_hand_force_torque_sensor_to_link;
marker_data.l_hand.num_params = sum(sum(isnan(l_markers)));
marker_data.l_hand.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, l_markers);

% Torso:
marker_data.utorso.num_markers = 6;
torso_markers = nan(3, marker_data.utorso.num_markers);
% #1 (middle of chest) can't be measured accurately due to the presumed
% inaccuracy of the chest shell mesh
torso_markers(:, 2) = ([0.286645; -0.118863; 0.607658] + [0.286462; -0.102694; 0.641651]) / 2 + [13e-3; 0; 0]; % highest one
% torso_markers(:, 5) = [0.217791; (-0.091985 + -0.133388) / 2; 0.536918] + [13e-3; 0; 24e-3]; % right vertical neck bar
% torso_markers(:, 6) = [0.148685; -0.255472; 0.526073] + [-10e-3; 10e-3; 10e-3]; % right shoulder
% torso_markers(:, 1) = [0.21796; (0.091728 + 0.133132) / 2; 0.536918] + [13e-3; 0; 24e-3]; % left vertical neck bar
% torso_markers(:, 4) = [0.148685; 0.255472; 0.526073] + [-10e-3; -10e-3; 10e-3]; % left shoulder
marker_data.utorso.num_params = sum(sum(isnan(torso_markers)));
marker_data.utorso.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, torso_markers);

% Right foot
l_foot_BCD_reference_x_mesh_error = 31.5e-3 - (0.12734 - 0.106719); % measured distance between off center protrusion and edge of flat part of foot minus that same distance in the mesh.
r_foot_A_reference = [0.117552; -0.063927; -0.034346];
r_foot_BCD_reference = [0.128065 + l_foot_BCD_reference_x_mesh_error; 0.06617; -0.034346];
marker_data.r_foot.num_markers = 4;
r_foot_markers = nan(3, marker_data.r_foot.num_markers);
r_foot_markers(:, 4) = r_foot_A_reference + [-78.5e-3; 12e-3; 12e-3]; %A
r_foot_markers(1:2, 4) = nan; % don't trust this measurement
r_foot_markers(:, 3) = r_foot_BCD_reference + [-50e-3; -80e-3; 16e-3]; %B
r_foot_markers(:, 2) = r_foot_BCD_reference + [-10e-3; -66e-3; 12e-3]; %C
r_foot_markers(:, 1) = r_foot_BCD_reference + [-50e-3; -50e-3; 16e-3]; %D
r_foot_markers(1:2, 1) = nan; % don't trust this measurement
marker_data.r_foot.num_params = sum(sum(isnan(r_foot_markers)));
marker_data.r_foot.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, r_foot_markers);

% Left foot
l_foot_ABC_reference_x_mesh_error = 31.5e-3 - (0.12734 - 0.106719); % measured distance between off center protrusion and edge of flat part of foot minus that same distance in the mesh.
l_foot_A_reference = [0.12734 + l_foot_ABC_reference_x_mesh_error; -0.065372; -0.033965];
l_foot_BC_reference = [0.12734 + l_foot_ABC_reference_x_mesh_error; 0.064725; -0.033965];
l_foot_D_reference = [0.116827; 0.064725; -0.033965];
marker_data.l_foot.num_markers = 4;
l_foot_markers = nan(3, marker_data.l_foot.num_markers);
l_foot_markers(:, 4) = l_foot_A_reference + [-58e-3; 25e-3; 16e-3]; %A
l_foot_markers(:, 3) = l_foot_BC_reference + [-42e-3; -70e-3; 16e-3]; %B
l_foot_markers(:, 2) = l_foot_BC_reference + [-50e-3; -30e-3; 16e-3]; %C
l_foot_markers(:, 1) = l_foot_D_reference + [-80e-3; -11e-3; 12e-3]; %D
l_foot_markers(1:2, 1) = nan; % don't trust this measurement.
marker_data.l_foot.num_params = sum(sum(isnan(l_foot_markers)));
marker_data.l_foot.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, l_foot_markers);

% Pelvis
marker_data.pelvis.num_markers = 5;
pelvis_markers = nan(3, marker_data.pelvis.num_markers);
pelvis_x = plate_thickness + ball_diameter / 2;
pelvis_marker_reference = [0.174772; 0; -0.014104]; % x: back of plate, y: center: z: bottom of bar/plate
pelvis_markers(:, 1) = pelvis_marker_reference + [pelvis_x; 9.5e-3; 89e-3];
pelvis_markers(:, 2) = pelvis_marker_reference + [pelvis_x; -48e-3; 51e-3];
pelvis_markers(:, 3) = pelvis_marker_reference + [pelvis_x; 9.5e-3; 32e-3];
pelvis_markers(:, 4) = pelvis_marker_reference + [pelvis_x; 47e-3; 70e-3];
pelvis_markers(:, 5) = pelvis_marker_reference + [pelvis_x; -9.5e-3; 127e-3];
pelvis_markers(1, :) = nan; % because I'm not sure that the plate was actually vertical
marker_data.pelvis.num_params = sum(sum(isnan(pelvis_markers)));
marker_data.pelvis.marker_positions = @(params) subsetOfMarkersMeasuredMarkerFunction(params, pelvis_markers);

end