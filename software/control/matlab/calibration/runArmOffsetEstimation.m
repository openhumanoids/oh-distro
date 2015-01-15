function dq = runArmOffsetEstimation(side, logfile, v_norm_limit, num_poses, torso_marker_function, torso_num_params, hand_marker_function, hand_num_params, options)
show_pose_indices = getOption(options, 'show_pose_indices', false);
show_data_synchronization = getOption(options, 'show_data_synchronization', false);
visualize_result = getOption(options, 'visualize_result', false);

% r = DRCAtlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
r = DRCAtlas();

[t_x,x_data,~,~,t_vicon,vicon_data,~,~,~,~,vicon_data_struct] = parseAtlasViconLog(r,logfile);

% time synchronization
t_x = t_x - t_x(1);
t_offset = min(t_x) - min(t_vicon) - 0;
t_vicon = t_vicon - t_x(1) + t_offset;  %%OFFSET TIME OF THE DATA! THIS MAY CHANGE

body_names = {'utorso'};
vicon_object_names = {'UTORSO'};

if strcmp(side, 'r')
  body_names{end + 1} = 'r_hand';
  vicon_object_names{end + 1} = 'RIGHTHAND';
elseif strcmp(side, 'l')
  body_names{end + 1} = 'l_hand';
  vicon_object_names{end + 1} = 'LEFTHAND';
else
  error('side not recognized');
end
marker_functions = {torso_marker_function, hand_marker_function};
marker_function_num_params = {torso_num_params, hand_num_params};
scales = {100, 1};

nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = length(body_names);

v_data = x_data(nq + (1 : nv), :);
pose_indices = findCalibrationPoseIndices(v_data, num_poses, v_norm_limit, show_pose_indices);
num_poses = length(pose_indices);

if show_data_synchronization
  vicon_dt = mean(diff(t_vicon));
  vicon_frequency = 1 / vicon_dt;
  filter = designfilt('lowpassiir', 'FilterOrder', 1, 'PassbandFrequency', 1, 'PassbandRipple', 1, 'SampleRate', vicon_frequency);
  
  vicon_diff = diff(vicon_data, 1, 2);
  vicon_dot = zeros(size(vicon_diff));
  for i = 1 : size(vicon_diff, 1)
    vicon_dot(i, :) = filtfilt(filter, vicon_diff(i, :));
  end
  vicon_dot_norm_limit = 20;
  pose_indices_vicon = findCalibrationPoseIndices(vicon_dot, num_poses, vicon_dot_norm_limit, true);

  figure();
  plot(t_vicon(pose_indices_vicon), zeros(size(pose_indices_vicon)), 'rx', t_x(pose_indices), zeros(size(pose_indices)), 'bo');
  title('time synchronization helper plot')
end

bodies = cell(nb, 1);
for i = 1 : nb
  bodies{i} = r.findLinkId(body_names{i});
end

[~, joint_indices] = r.findKinematicPath(bodies{1}, bodies{2});
q_indices = [r.getBody(joint_indices).position_num];
q_data = zeros(nq, num_poses);

motion_capture_data = cell(nb, 1);
for i = 1 : num_poses
  x_pose_index = pose_indices(i);
  vicon_pose_index = find(t_vicon > t_x(x_pose_index),1);
  q_data(:,i) = x_data(1 : nq, x_pose_index);
  
  for j = 1 : nb
    raw_vicon_data = getViconObjectData(vicon_data_struct, vicon_object_names{j});
    motion_capture_data{j}(:, :, i) = raw_vicon_data(1:3,:,vicon_pose_index);
    occluded_markers = logical(raw_vicon_data(4,:,vicon_pose_index));
    motion_capture_data{j}(:, occluded_markers, i) = nan(3, sum(occluded_markers));
  end
end

options.search_floating = true;
[dq, marker_params, floating_states, objective_value, marker_residuals, info] = jointOffsetCalibration(r, q_data, q_indices,...
    bodies, marker_functions, marker_function_num_params, motion_capture_data, scales, options);

dq = unwrap([0;dq]);
dq = dq(2:end);

if visualize_result
  for pose_num = 1 : num_poses
    v = r.constructVisualizer;
    lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'arm_offset_estimation');
    q = q_data(:, pose_num);
    q(1:6) = floating_states(:,pose_num);
    q(q_indices) = q(q_indices) + dq;
    v.draw(0, q);
    
    kinsol = r.doKinematics(q);
    torso_pts = r.forwardKin(kinsol,torso_body,torso_marker_function(marker_params{1}));
    torso_pts_fixed = torso_marker_function(nan(torso_num_params, 1));
    hand_pts = r.forwardKin(kinsol,hand_body,hand_marker_function(marker_params{2}));
    hand_pts_fixed = hand_marker_function(nan(hand_num_params, 1));
    
    all_pts = [torso_pts hand_pts];
    all_pts_fixed = [torso_pts_fixed hand_pts_fixed];
    all_data = [torso_data hand_data];
    
    for i=1:size(all_pts,2),
      % marker positions in body frame, transformed to world frame
      if all(isnan(all_pts_fixed(:, i)))
        % none of the coordinates in body frame measured: color red
        lcmgl.glColor3f(1,0,0);
      else
        % some of the coordinates in body frame measured: color green
        % the brighter the color, the more coordinates are known in body
        % frame
        green_value = sum(~isnan(all_pts_fixed(:, i))) / size(all_pts_fixed, 1);
        lcmgl.glColor3f(0,green_value,0);
      end
      lcmgl.sphere(all_pts(:,i),.01,20,20);
      
      % vicon marker position
      % blue
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(all_data(:,i,pose_num),.01,20,20);
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
  ret = default;
end
end
