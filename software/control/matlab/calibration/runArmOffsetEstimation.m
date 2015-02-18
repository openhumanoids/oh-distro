function dq = runArmOffsetEstimation(side, logfile, marker_info_struct, options)
body_names = {'utorso'};
vicon_object_names = {'TORSO'};

if strcmp(side, 'r')
  body_names{end + 1} = 'r_hand';
  vicon_object_names{end + 1} = 'R_HAND';
elseif strcmp(side, 'l')
  body_names{end + 1} = 'l_hand';
  vicon_object_names{end + 1} = 'L_HAND';
else
  error('side not recognized');
end

marker_info_struct.(body_names{1}).scale = 100;
marker_info_struct.(body_names{2}).scale = 1;
options.calibration_type = 'offset';

dq = runJointCalibration(logfile, body_names, vicon_object_names, marker_info_struct, options);

end
