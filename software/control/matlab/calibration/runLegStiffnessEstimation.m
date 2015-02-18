function k = runLegStiffnessEstimation(logfile, marker_info_struct, options)
body_names = {'pelvis', 'r_foot', 'l_foot'};

vicon_object_names = {'FRONTPLATE', 'RIGHTFOOT', 'LEFTFOOT'};

% for i = 1 : length(body_names)
%   marker_info_struct.(body_names{i}).scale = 1;
% end
marker_info_struct.pelvis.scale = 100;
marker_info_struct.r_foot.scale = 1;
marker_info_struct.l_foot.scale = 1;

options.calibration_type = 'stiffness';

k = runJointCalibration(logfile, body_names, vicon_object_names, marker_info_struct, options);

end