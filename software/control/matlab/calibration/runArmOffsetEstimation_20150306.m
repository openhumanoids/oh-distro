function runArmOffsetEstimation_20150306()
measurements_dir = fullfile(getenv('DRC_PATH'), 'control', 'matlab', 'calibration', 'viconMarkerMeasurements');
addpath(measurements_dir)

options.show_pose_indices = true;
options.show_data_synchronization = false;
options.visualize_result = true;
options.v_norm_limit = 0.5; %0.05;
options.num_poses = 24;
options.use_extra_data = true;
options.just_visualize_vicon = false;

marker_data = measuredViconMarkerPositions();

% logfile_left = strcat(getenv('DRC_PATH'),'/../../logs/useful/arm-offset-calibration-20150306/lcmlog-2015-03-06.left');
% runArmOffsetEstimation('l', logfile_left, marker_data, options);

logfile_right = strcat(getenv('DRC_PATH'),'/../../logs/useful/arm-offset-calibration-20150306/lcmlog-2015-03-06.right');
runArmOffsetEstimation('r', logfile_right, marker_data, options);

rmpath(measurements_dir);
end