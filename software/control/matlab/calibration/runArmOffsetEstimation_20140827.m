function runArmOffsetEstimation_20140827()
measurements_dir = fullfile(getenv('DRC_PATH'), 'control', 'matlab', 'calibration', 'viconMarkerMeasurements');
addpath(measurements_dir)

options.show_pose_indices = false;
options.show_data_synchronization = false;
options.visualize_result = false;

v_norm_limit = 0.05;
num_poses = 20;
marker_data = measuredViconMarkerPositions();

logfile_left = strcat(getenv('DRC_PATH'),'/../logs/arm_calibration/lcmlog-2014-08-27.03-vicon-left-arm-offsets');
dq_left = runArmOffsetEstimation('l', logfile_left, v_norm_limit, num_poses, ...
  marker_data.utorso.marker_positions, marker_data.utorso.num_params, ...
  marker_data.l_hand.marker_positions, marker_data.l_hand.num_params, options);
disp('left offsets:')
disp(dq_left);

logfile_right = strcat(getenv('DRC_PATH'),'/../logs/arm_calibration/lcmlog-2014-08-27.06-vicon-right-arm-offsets');
dq_right = runArmOffsetEstimation('r', logfile_right, v_norm_limit, num_poses, ...
  marker_data.utorso.marker_positions, marker_data.utorso.num_params, ...
  marker_data.r_hand.marker_positions, marker_data.r_hand.num_params, options);
disp('right offsets:')
disp(dq_right)

rmpath(measurements_dir);
end