function q_correction_params = runJointCalibration(logfile, body_names, vicon_object_names, marker_info_struct, options)

show_pose_indices = getOption(options, 'show_pose_indices', false);
show_data_synchronization = getOption(options, 'show_data_synchronization', false);
visualize_result = getOption(options, 'visualize_result', false);
vicon_lag = getOption(options, 'vicon_lag', 0);
calibration_type = getOption(options, 'calibration_type');
v_norm_limit = getOption(options, 'v_norm_limit');
num_poses = getOption(options, 'num_poses');

% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
r = Atlas();
[t_x,x_data,t_u,u_data_full,t_vicon,vicon_data,~,~,~,~,vicon_data_struct] = parseAtlasViconLog(r,logfile);
u_data_full = u_data_full(end - r.getNumInputs + 1 : end, :); % the last num_u rows of u_data are the actual torques

% time synchronization
t_u = t_u - t_x(1);
t_offset = min(t_x) - min(t_vicon) - vicon_lag;
t_vicon = t_vicon - t_x(1) + t_offset;
t_x = t_x - t_x(1);

nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = length(body_names);

marker_functions = cell(nb, 1);
marker_function_num_params = cell(nb, 1);
scales = cell(nb, 1);
for i = 1 : length(body_names)
  body_name = body_names{i};
  marker_functions{i} = marker_info_struct.(body_name).marker_positions;
  marker_function_num_params{i} = marker_info_struct.(body_name).num_params;
  scales{i} = marker_info_struct.(body_name).scale;
end

q_data_full = x_data(1:nq, :);
v_data_full = x_data(nq + (1 : nv), :);
pose_indices = findCalibrationPoseIndices(v_data_full, num_poses, v_norm_limit, show_pose_indices);
num_poses = length(pose_indices);

if show_data_synchronization
  vicon_dot_norm_limit = 20;
  showViconTimeSynchronization(t_x, t_vicon, vicon_data, vicon_dot_norm_limit, pose_indices);
end

bodies = cell(nb, 1);
for i = 1 : nb
  bodies{i} = r.findLinkInd(body_names{i});
end

joint_indices = [];
for i = 2 : nb
  [~, additional_joint_indices] = r.findKinematicPath(bodies{1}, bodies{i});
  joint_indices = [joint_indices additional_joint_indices];
end
q_indices = [r.getBody(joint_indices).position_num];

[q_data, motion_capture_data, u_data] = selectPoseData(t_x, t_vicon, t_u, q_data_full, vicon_data_struct, vicon_object_names, u_data_full, pose_indices);

options.search_floating = true;

if strcmp(calibration_type, 'offset')
  [dq, marker_params, floating_states, objective_value, marker_residuals, info] = jointOffsetCalibration(r, q_data, q_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options);
  dq = unwrap([0;dq]);
  dq = dq(2:end);
  q_correction_params = dq;
  
  q_data(1:6, :) = floating_states;
  q_data(q_indices, :) = q_data(q_indices, :) + repmat(dq, [1 num_poses]);
end

if strcmp(calibration_type, 'stiffness')
  k_initial = getOption(options, 'k_initial');
  [k, marker_params, floating_states, objective_value, marker_residuals, info] = jointStiffnessCalibration(r, q_data, u_data, q_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, k_initial, options);
  
  q_correction_params = k;
  
  q_data(1:6, :) = floating_states;
%   B = r.getB();
%   B_calibrated_joints = B(q_indices, :);
%   [rows, u_indices] = find(B_calibrated_joints ~= 0);
%   [~, sort_indices] = sort(rows);
%   u_indices = u_indices(sort_indices);
%   tau_data = B(q_indices, u_indices) * u_data(u_indices, :);
%   K = diag(k);
%   q_data(q_indices, :) = q_data(q_indices, :) + K \ tau_data;
  
end


if visualize_result
  visualizeCalibrationResult(r, bodies, q_data, marker_functions, marker_params, motion_capture_data);
end
end

function vicon_object_data = getViconObjectData(vicon_data_struct, vicon_object_name)
vicon_object_index = cellfun(@(x) strcmp(x.name, vicon_object_name), vicon_data_struct);
vicon_object_data = vicon_data_struct{vicon_object_index}.data;
vicon_object_data(1:3,:,:) = vicon_object_data(1:3,:,:)/1e3;
end

function ret = getOption(options, fieldname, default)
if isfield(options, fieldname)
  ret = options.(fieldname);
else
  if nargin < 3
    error(['required option ' fieldname ' not found']);
  else
    ret = default;
  end
end
end

function showViconTimeSynchronization(t_x, t_vicon, vicon_data, vicon_dot_norm_limit, pose_indices)
vicon_dt = mean(diff(t_vicon));
vicon_frequency = 1 / vicon_dt;
filter = designfilt('lowpassiir', 'FilterOrder', 1, 'PassbandFrequency', 1, 'PassbandRipple', 1, 'SampleRate', vicon_frequency);

vicon_diff = diff(vicon_data, 1, 2);
vicon_dot = zeros(size(vicon_diff));
for i = 1 : size(vicon_diff, 1)
  vicon_dot(i, :) = filtfilt(filter, vicon_diff(i, :));
end
num_poses = length(pose_indices);
pose_indices_vicon = findCalibrationPoseIndices(vicon_dot, num_poses, vicon_dot_norm_limit, true);

figure();
plot(t_vicon(pose_indices_vicon), zeros(size(pose_indices_vicon)), 'rx', t_x(pose_indices), zeros(size(pose_indices)), 'bo');
title('time synchronization helper plot')
end

function [q_data, motion_capture_data, u_data] = selectPoseData(t_x, t_vicon, t_u, q_data_full, vicon_data_struct, vicon_object_names, u_data_full, pose_indices)
nq = size(q_data_full, 1);
nb = length(vicon_object_names);
nu = size(u_data_full, 1);
num_poses = length(pose_indices);

q_data = zeros(nq, num_poses);
motion_capture_data = cell(nb, 1);
u_data = zeros(nu, num_poses);
for i = 1 : num_poses
  x_pose_index = pose_indices(i);
  vicon_pose_index = find(t_vicon > t_x(x_pose_index),1);
  u_index = find(t_u > t_x(x_pose_index),1);
  
  q_data(:,i) = q_data_full(:, x_pose_index);
  
  for j = 1 : nb
    raw_vicon_data = getViconObjectData(vicon_data_struct, vicon_object_names{j});
    motion_capture_data{j}(:, :, i) = raw_vicon_data(1:3,:,vicon_pose_index);
    occluded_markers = logical(raw_vicon_data(4,:,vicon_pose_index));
    motion_capture_data{j}(:, occluded_markers, i) = nan(3, sum(occluded_markers));
  end
  
  u_data(:, i) = u_data_full(:, u_index);
end
end

function visualizeCalibrationResult(r, bodies, q_data, marker_functions, marker_params, motion_capture_data)
num_poses = size(q_data, 2);
nb = length(bodies);
for pose_num = 1 : num_poses
  v = r.constructVisualizer;
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'calibration_result_visualization');
  q = q_data(:, pose_num);
  v.draw(0, q);
  
  kinsol = r.doKinematics(q);
  
  for j = 1 : nb
    pts = r.forwardKin(kinsol, bodies{j}, marker_functions{j}(marker_params{j}));
    measured_coordinates_mask = isnan(marker_functions{j}(nan(numel(marker_params{j}), 1)));
    pts_vicon = motion_capture_data{j};
    
    for i=1:size(pts, 2),
      % marker positions in body frame, transformed to world frame
      if all(measured_coordinates_mask(:, i))
        % none of the coordinates in body frame measured: color red
        lcmgl.glColor3f(1,0,0);
      else
        % some of the coordinates in body frame measured: color green
        % the brighter the color, the more coordinates are known in body
        % frame
        green_value = sum(~measured_coordinates_mask(:, i)) / size(measured_coordinates_mask, 1);
        lcmgl.glColor3f(0,green_value,0);
      end
      
      marker_label = [r.getBody(bodies{j}).linkname ' ' num2str(i)];
      lcmgl.sphere(pts(:,i),.01,20,20);
      lcmgl.glColor3f(1,1,1);
      lcmgl.text(pts(:, i), marker_label);
      
      % vicon marker position
      % blue
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pts_vicon(:,i,pose_num),.01,20,20);
      lcmgl.glColor3f(1,1,1);
      lcmgl.text(pts_vicon(:,i,pose_num), [marker_label ' vicon']);
    end
  end
  
  lcmgl.switchBuffers();
  
  if pose_num < num_poses
    reply = input('Visualize next pose? y/n [y]:','s');
    if isempty(reply)
      reply = 'y';
    end
    if strcmp(reply, 'n')
      break;
    end
  end
end
end