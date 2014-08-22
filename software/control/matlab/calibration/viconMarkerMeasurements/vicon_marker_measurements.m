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

% Left hand:
l_reference = [0.066335; 0.06481; -0.020503]; % obtained from meshlab
l_reference_to_base = [-0.017; 0.013; 0]; % measured offset of base w.r.t. reference
l_base = l_reference + l_reference_to_base;
l_base_to_marker = [0; 0; -(standoff_offset + standoff_length + ball_diameter / 2)];
l_marker = l_base + l_base_to_marker;