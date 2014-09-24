function runLegStiffnessEstimation_20140822()
measurements_dir = fullfile(getenv('DRC_PATH'), 'control', 'matlab', 'calibration', 'viconMarkerMeasurements');

options.show_pose_indices = true;
options.show_data_synchronization = false;
options.visualize_result = true;
options.v_norm_limit = 0.05;
options.num_poses = 18;
options.k_initial = 1000 * ones(12, 1) + 10 * rand;

addpath(measurements_dir)
marker_data = measuredViconMarkerPositions();
rmpath(measurements_dir);

logfile = strcat(getenv('DRC_PATH'),'/../logs/leg_calibration/lcmlog-2014-08-22.00_feet_stationary_moving_around');
k = runLegStiffnessEstimation(logfile, marker_data, options);

end