function q_correction_params = runJointCalibration(logfile, body_names, vicon_object_names, marker_info_struct, options)

show_pose_indices = getOption(options, 'show_pose_indices', false);
visualize_result = getOption(options, 'visualize_result', false);
vicon_lag = getOption(options, 'vicon_lag', 0);
calibration_type = getOption(options, 'calibration_type');
v_norm_limit = getOption(options, 'v_norm_limit');
num_poses = getOption(options, 'num_poses');
use_extra_data = getOption(options, 'use_extra_data', false);
just_visualize_vicon = getOption(options, 'just_visualize_vicon', false);
min_time = getOption(options, 'min_time', -inf);
max_time = getOption(options, 'max_time', inf);

% r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
r = DRCAtlas();

if exist([logfile '.mat'], 'file')
  load([logfile '.mat']);
else
  [t_x,x_data,t_u,u_data_full,t_vicon,vicon_data_full,~,~,t_extra,extra_data,vicon_data_struct] = parseAtlasViconLog(r,logfile);
  save([logfile '.mat'], 't_x', 'x_data', 't_u', 'u_data_full', 't_vicon', 'vicon_data_full', 't_extra', 'extra_data', 'vicon_data_struct');
end

u_data_full = u_data_full(end - r.getNumInputs + 1 : end, :); % the last num_u rows of u_data are the actual torques

% time synchronization
t_u = t_u - t_x(1);
t_offset = min(t_x) - min(t_vicon) - vicon_lag;
t_vicon = t_vicon - t_x(1) + t_offset;
t_x = t_x - t_x(1);
t_extra = t_extra - t_extra(1);

% trim data
min_index = find(t_x > min_time, 1); if isempty(min_index), min_index = 1; end;
max_index = find(t_x > max_time, 1); if isempty(max_index), max_index = length(t_x); end;

t_x = t_x(min_index : max_index);
x_data = x_data(:, min_index : max_index);

nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = length(body_names);

marker_functions = cell(nb, 1);
marker_function_num_params = cell(nb, 1);
num_markers = cell(nb, 1);
scales = cell(nb, 1);
for i = 1 : length(body_names)
  body_name = body_names{i};
  marker_functions{i} = marker_info_struct.(body_name).marker_positions;
  marker_function_num_params{i} = marker_info_struct.(body_name).num_params;
  num_markers{i} = marker_info_struct.(body_name).num_markers;
  scales{i} = marker_info_struct.(body_name).scale;
end

q_data_full = x_data(1:nq, :);
v_data_full = x_data(nq + (1 : nv), :);

nrevolute_joints = length([r.getManipulator.body(3:end).position_num]);
q_data_extra = extra_data(1 : nrevolute_joints, :);
v_data_extra = extra_data(nrevolute_joints + (1 : nrevolute_joints), :);
extra_data_valid_indices = ~all(q_data_extra == 0, 2);

if use_extra_data
  q_data_full(r.stateToBDIInd(extra_data_valid_indices), :) = interp1(t_extra, q_data_extra(extra_data_valid_indices, :)', t_x)';
  v_data_full(r.stateToBDIInd(extra_data_valid_indices), :) = interp1(t_extra, v_data_extra(extra_data_valid_indices, :)', t_x)';
end

bodies = cell(nb, 1);
for i = 1 : nb
  bodies{i} = r.findLinkId(body_names{i});
end

joint_indices = [];
for i = 2 : nb
  [~, additional_joint_indices] = r.findKinematicPath(bodies{1}, bodies{i});
  joint_indices = [joint_indices; additional_joint_indices]; %#ok<AGROW>
end
q_indices = [r.getBody(joint_indices).position_num];
v_indices = [r.getBody(joint_indices).velocity_num];

pose_indices = findCalibrationPoseIndices(v_data_full(v_indices, :), num_poses, v_norm_limit);
if show_pose_indices
  showPoseIndices(t_x, v_data_full(v_indices, :), pose_indices);
end
num_poses = length(pose_indices);

[q_data, motion_capture_data, u_data] = selectPoseData(t_x, t_vicon, t_u, q_data_full, vicon_data_struct, vicon_object_names, u_data_full, pose_indices);

if just_visualize_vicon
  visualizeCalibrationResult(r, bodies, q_data, [], [], [], motion_capture_data, num_markers);
  return;
end

options.search_floating = 'full';

if strcmp(calibration_type, 'offset')
  [dq, marker_params, floating_states, objective_value, marker_residuals, info] = jointOffsetCalibration(r, q_data, q_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options);
  dq = unwrap([0;dq]);
  dq = dq(2:end);
  q_correction_params = dq;
end

if strcmp(calibration_type, 'stiffness')
  k_initial = getOption(options, 'k_initial');
  [k, marker_params, floating_states, objective_value, marker_residuals, info] = jointStiffnessCalibration(r, q_data, u_data, q_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, k_initial, options);
  q_correction_params = k;
end

q_data([1:2 6], :) = floating_states([1:2 6], :); % take xy and yaw from calibration routine
q_data_after = q_data;
q_data_after(1:6, :) = floating_states;

if strcmp(calibration_type, 'offset')
  q_data_after(q_indices, :) = q_data(q_indices, :) + repmat(dq, [1 num_poses]);
end

if strcmp(calibration_type, 'stiffness')
  B = r.getB();
  B_calibrated_joints = B(q_indices, :);
  [rows, u_indices] = find(B_calibrated_joints ~= 0);
  [~, sort_indices] = sort(rows);
  u_indices = u_indices(sort_indices);
  tau_data = B(q_indices, u_indices) * u_data(u_indices, :);
  K = diag(k);
  q_data_after(q_indices, :) = q_data(q_indices, :) - K \ tau_data;
end

fprintf([calibration_type ' calibration results:\n']);
for i = 1 : length(joint_indices)
  fprintf([r.getBody(joint_indices(i)).jointname ': %0.4g\n'], q_correction_params(i));
end

if visualize_result
  visualizeCalibrationResult(r, bodies, q_data, q_data_after, marker_functions, marker_params, motion_capture_data, num_markers);
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

function showPoseIndices(t_x, v_data_full, pose_indices)
figure();
v_norm = sum(sqrt(v_data_full .* v_data_full), 1);
hold on;
plot(t_x, v_norm / max(v_norm), 'b');

plot(t_x(pose_indices), zeros(size(pose_indices)), 'r*');

legend({'v norm (normalized)', 'udot norm (normalized)', 'vicondot norm (normalized)', 'selected pose indices'});
hold off;
end

function visualizeCalibrationResult(r, bodies, q_data_before, q_data_after, marker_functions, marker_params, motion_capture_data, num_markers)
num_poses = size(q_data_before, 2);
nb = length(bodies);
show_before = false;
pose_num = 1;
first_iteration = true;
while true
  if ~first_iteration
    if show_before
      other_view = 'after';
    else
      other_view = 'before';
    end
    reply = input(['Visualize next pose: y/n [y] or switch to view ' other_view ' calibration: s?'],'s');
    if isempty(reply)
      reply = 'y';
    end
    if strcmp(reply, 'y')
      pose_num = pose_num + 1;
    end
    if strcmp(reply, 'n')
      break;
    end
    if strcmp(reply, 's')
      show_before = ~show_before;
    end
  end
  first_iteration = false;
  if pose_num > num_poses
    break;
  end
  
  v = r.constructVisualizer;
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'calibration_result_visualization');
  if show_before || isempty(q_data_after)
    q_data = q_data_before;
  else
    q_data = q_data_after;
  end
  q = q_data(:, pose_num);
  v.draw(0, q);
  
  kinsol = r.doKinematics(q);
  
  for j = 1 : nb
    if ~isempty(marker_functions)
      pts = r.forwardKin(kinsol, bodies{j}, marker_functions{j}(marker_params{j}));
      measured_coordinates_mask = isnan(marker_functions{j}(nan(numel(marker_params{j}), 1)));
    end
    pts_vicon = motion_capture_data{j};
    
    for i = 1 : num_markers{j}
      marker_label = num2str(i); %[r.getBody(bodies{j}).linkname ' ' num2str(i)];
      % marker positions in body frame, transformed to world frame
      if ~isempty(marker_functions)
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
        lcmgl.sphere(pts(:,i),.01,20,20);
        lcmgl.glColor3f(1,1,1);
        lcmgl.text(pts(:, i), marker_label);
      end
      
      % vicon marker position
      % blue
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pts_vicon(:,i,pose_num),.01,20,20);
      lcmgl.glColor3f(1,1,1);
      lcmgl.text(pts_vicon(:,i,pose_num), [marker_label ' vicon']);
    end
  end
  
  lcmgl.switchBuffers();
end
end