function runArmOffsetEstimation_20150212()
measurements_dir = fullfile(getenv('DRC_PATH'), 'control', 'matlab', 'calibration', 'viconMarkerMeasurements');
addpath(measurements_dir)

options.show_pose_indices = true;
options.show_data_synchronization = false;
options.visualize_result = true;
options.v_norm_limit = 0.5; %0.05;
options.num_poses = 20;
options.use_extra_data = true;

marker_data = measuredViconMarkerPositions();

% logfile_left = strcat(getenv('DRC_PATH'),'/../../logs/useful/arm-offset-calibration-20150212/lcmlog-2015-02-12.left');
% dq_left = runArmOffsetEstimation('l', logfile_left, marker_data, options);
% disp('left offsets:')
% disp(dq_left);

logfile_right = strcat(getenv('DRC_PATH'),'/../../logs/useful/arm-offset-calibration-20150212/lcmlog-2015-02-12.right');
dq_right = runArmOffsetEstimation('r', logfile_right, marker_data, options);
disp('right offsets:')
disp(dq_right)

rmpath(measurements_dir);
end