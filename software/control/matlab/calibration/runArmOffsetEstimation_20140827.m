function runArmOffsetEstimation_20140827()
logfile_left = strcat(getenv('DRC_PATH'),'/../logs/arm_calibration/lcmlog-2014-08-27.03-vicon-left-arm-offsets');
logfile_right = strcat(getenv('DRC_PATH'),'/../logs/arm_calibration/lcmlog-2014-08-27.06-vicon-right-arm-offsets');
v_norm_limit = 0.05;
num_torso_markers = 3;
num_hand_markers = 5;

dq_left = runArmOffsetEstimation('l', logfile_left, v_norm_limit, 20, @torsoMarkerFun, num_torso_markers, @leftHandMarkerFun, num_hand_markers);
dq_right = runArmOffsetEstimation('r', logfile_right, v_norm_limit, 20, @torsoMarkerFun, num_torso_markers, @rightHandMarkerFun, num_hand_markers);
end

function [x, dx] = torsoMarkerFun(params)
num_torso_markers = 3;
marker_positions_measured = nan(3, num_torso_markers);
% TODO: fill in measured markers
[x, dx] = subsetOfMarkersMeasuredMarkerFunction(params, marker_positions_measured);
end

function [x, dx] = leftHandMarkerFun(params)
num_torso_markers = 5;
marker_positions_measured = nan(3, num_torso_markers);
% TODO: fill in measured markers
[x, dx] = subsetOfMarkersMeasuredMarkerFunction(params, marker_positions_measured);
end

function [x, dx] = rightHandMarkerFun(params)
num_torso_markers = 5;
marker_positions_measured = nan(3, num_torso_markers);
% TODO: fill in measured markers
[x, dx] = subsetOfMarkersMeasuredMarkerFunction(params, marker_positions_measured);
end