function dq = runArmOffsetEstimation(side, logfile, v_norm_limit, num_poses, torso_marker_function, torso_num_params, hand_marker_function, hand_num_params)
%NOTEST

show_pose_indices = true;
show_data_synchronization = true;
visualize_result = false;

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

[t_x,x_data,~,~,t_vicon,vicon_data,~,~,~,~,vicon_data_struct] = parseAtlasViconLog(r,logfile);

% time synchronization
t_x = t_x - t_x(1);
t_offset = min(t_x) - min(t_vicon) - 0;
t_vicon = t_vicon - t_x(1) + t_offset;  %%OFFSET TIME OF THE DATA! THIS MAY CHANGE

torso_body_name = 'utorso';
torso_vicon_object_name = 'UTORSO';

if strcmp(side, 'r')
  hand_body_name = 'r_hand';
  hand_vicon_object_name = 'RIGHTHAND';
elseif strcmp(side, 'l')
  hand_body_name = 'l_hand';
  hand_vicon_object_name = 'LEFTHAND';
else
  error('side not recognized');
end

nq = r.getNumPositions();
nv = r.getNumVelocities();

v_data = x_data(nq + (1 : nv), :);
pose_indices = findCalibrationPoseIndices(v_data, num_poses, v_norm_limit, show_pose_indices);

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

torso_body = r.findLinkInd(torso_body_name);
hand_body = r.findLinkInd(hand_body_name);

[~, joint_indices] = r.findKinematicPath(torso_body, hand_body);
q_indices = [r.getBody(joint_indices).position_num];

torso_markers = getViconObjectData(vicon_data_struct, torso_vicon_object_name);
hand_markers = getViconObjectData(vicon_data_struct, hand_vicon_object_name);

q_data = zeros(nq, num_poses);
torso_data = zeros(3, size(torso_markers, 2), num_poses);
hand_data = zeros(3, size(hand_markers, 2), num_poses);
for i = 1 : num_poses
  x_pose_index = pose_indices(i);
  q_data(:,i) = x_data(1 : nq, x_pose_index);
  
  vicon_pose_index = find(t_vicon > t_x(x_pose_index),1);
  torso_data(:,:,i) = torso_markers(1:3,:,vicon_pose_index);
  hand_data(:,:,i) = hand_markers(1:3,:,vicon_pose_index);
end

[dq, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = ...
  jointOffsetCalibration(r, q_data, q_indices,torso_body, torso_marker_function, torso_num_params, torso_data, hand_body, hand_marker_function, hand_num_params, hand_data);
dq = unwrap([0;dq]);
dq = dq(2:end);

if visualize_result
  v = r.constructVisualizer;
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');
  j = 1; % TODO
  q = zeros(nq, 1);
  q(1:6) = floating_states(:,j);
  q(q_indices) = q(q_indices) + dq;
  v.draw(0,q);
  
  kinsol = r.doKinematics(q);
  handpts = r.forwardKin(kinsol,hand_body,rightHandMarkerPos_20131123(body2_params));
  torsopts = r.forwardKin(kinsol,torso_body,torsoMarkerPos_20131123(body1_params,true));
  % data 3 = function 4
  for i=1:size(torso_markers,2),
    lcmgl.glColor3f(1,0,0); % red
    lcmgl.sphere(torsopts(:,i),.01,20,20);
    lcmgl.glColor3f(0,0,1); % blue
    lcmgl.sphere(torso_data(:,i,j),.01,20,20);
    
  end
  
  for i=1:size(hand_markers,2),
    lcmgl.glColor3f(1,0,0); % red
    lcmgl.sphere(handpts(:,i),.01,20,20);
    lcmgl.glColor3f(0,0,1); % blue
    lcmgl.sphere(hand_data(:,i,j),.01,20,20);
  end
  
  lcmgl.switchBuffers();
  
  sprintf('mean err (mm): %d',mean(sqrt(sum(body2_resids_check.*body2_resids_check)))*1000)
end
end

function vicon_object_data = getViconObjectData(vicon_data_struct, vicon_object_name)
vicon_object_index = cellfun(@(x) strcmp(x.name, vicon_object_name), vicon_data_struct);
vicon_object_data = vicon_data_struct{vicon_object_index}.data;
vicon_object_data(1:3,:,:) = vicon_object_data(1:3,:,:)/1e3;
end
