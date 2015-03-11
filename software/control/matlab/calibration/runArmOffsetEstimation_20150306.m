function runArmOffsetEstimation_20150306()
measurements_dir = fullfile(getenv('DRC_PATH'), 'control', 'matlab', 'calibration', 'viconMarkerMeasurements');
addpath(measurements_dir)

options.show_pose_indices = true;
options.visualize_result = true;
options.use_extra_data = true;
options.just_visualize_vicon = false;

marker_data = measuredViconMarkerPositions();

options.v_norm_limit = 0.1;
options.num_poses = 24;
options.max_time = 342;
logfile_left = strcat(getenv('DRC_PATH'),'/../../../logs/useful/arm-offset-calibration-20150306/lcmlog-2015-03-06.left');
runArmOffsetEstimation('l', logfile_left, marker_data, options);

options.v_norm_limit = 0.1;
options.num_poses = 24;
options.max_time = 397;
logfile_right = strcat(getenv('DRC_PATH'),'/../../../logs/useful/arm-offset-calibration-20150306/lcmlog-2015-03-06.right');
runArmOffsetEstimation('r', logfile_right, marker_data, options);

rmpath(measurements_dir);
end
