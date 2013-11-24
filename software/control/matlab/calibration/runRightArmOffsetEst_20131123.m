  %NOTEST
if ~exist('r')
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
end
% logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-02-vicon-test');
% logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-07.00_right_arm_calib');

% logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-09.00_right_arm_vicon_calibration');
% t_offset = -.56;
% lcmlog-2013-11-23.00_vicon
logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-11-23.00_vicon');
[t_x,x_data,t_u,u_data,t_vicon,vicon_data,state_frame,input_frame,t_extra,extra_data,vicon_data_struct] = parseAtlasViconLog(r,logfile);

t_offset = min(t_x) - min(t_vicon) - .1;


t_u = t_u - t_x(1);
t_vicon = t_vicon - t_x(1) + t_offset;  %%OFFSET TIME OF THE DATA! THIS MAY CHANGE
t_extra = t_extra - t_x(1);
t_x = t_x - t_x(1);

joint_indices = [22:26 33];
% joint_indices = [];
x0_offset = -extra_data(22+(1:6),1)+x_data(joint_indices((1:6)),1);
% hack to use encoder data
x_data_bkp = x_data;
for i=1:6,
  x_data(joint_indices(i),:) = (extra_data(22+i,:))-extra_data(22+i,1)+x_data(joint_indices(i),1);
end

% joint_indices = [33];

% Sample times
% todo, maybe filter the data around them?

% t_sample = [3 16 20.7 31 56.9 64.5];  
% t_sample  = [3 16 20.7 31 56.9 64.5 88.3 100.9];
% t_sample = [20.7 31 56.9 64.5 88.3 100.9 120 131.4 133.5 143.5 147.9 155.6];
% t_sample = [120 131.4 133.5 143.5 147.9 155.6];
% t_sample = [20.7];

%%
% t_sample = [2.5 9.8 22.5 27.4 38.5 47.5];
% t_sample = 9.8;
% t_sample = [9.8 38.5 45.7];
% t_sample = [2.5 9.8 22.5 38.5 47.5 71.5 83.4 93.4];

% for 10-24 data
% t_sample = [55 110 144 178.7 215 245 333 364.4 395 424.4 492.1 518.7 540.8];
% t_sample = [88 111.8 150.5 171.1 187.8 204 244.3];
t_sample = [40.07 51.33 56.52 60.57 65.3];
t_sample = [22.57  35.65 78 113.4 132 155 172 223.8];

% torso_markers = reshape(vicon_data(21:40,:),4,5,length(t_vicon));
% hand_markers = reshape(vicon_data(1:20,:),4,5,length(t_vicon));
torso_markers = vicon_data_struct{2}.data;
hand_markers = vicon_data_struct{4}.data;

torso_markers(1:3,:,:) = torso_markers(1:3,:,:)/1e3;
hand_markers(1:3,:,:) = hand_markers(1:3,:,:)/1e3;

left_hand_markers = vicon_data_struct{3}.data(1:3,:,:)/1e3;
pelvis_markers = vicon_data_struct{1}.data(1:3,:,:)/1e3;



% hand_markers = hand_markers - reshape(repmat([mean(mean(torso_markers(1:3,:,:),2),3);0],size(hand_markers,2)*size(hand_markers,3),1),4,size(hand_markers,2),[]);
% torso_markers = torso_markers - reshape(repmat([mean(mean(torso_markers(1:3,:,:),2),3);0],size(torso_markers,2)*size(torso_markers,3),1),4,size(torso_markers,2),[]);

% %% change coordinates to make this a little easier
% --doesn't seem to really matter much
% tmp = hand_markers;
% hand_markers(1,:,:) = -tmp(3,:,:);
% hand_markers(2,:,:) = -tmp(1,:,:);
% hand_markers(3,:,:) =  tmp(2,:,:);
% 
% tmp = torso_markers;
% torso_markers(1,:,:) = -tmp(3,:,:);
% torso_markers(2,:,:) = -tmp(1,:,:);
% torso_markers(3,:,:) =  tmp(2,:,:);


torso_body = 5;
hand_body = 29;

clear q_data torso_data hand_data
% filter_len = 11;
% x_data = filter(ones(filter_len,1)/filter_len,1,x_data);
% torso_markers(1:3,:,:) = filter(ones(filter_len,1)/filter_len,1,torso_markers(1:3,:,:));
% hand_markers(1:3,:,:) = filter(ones(filter_len,1)/filter_len,1,hand_markers(1:3,:,:));
% R = rpy2ro  tmat([-1.4192;-4.0072; 1.6042]);
R = eye(3);
avg_range = 0;
for i=1:length(t_sample),
  ind_i = find(t_x > t_sample(i),1);
%   q_data(:,i) = x_data(1:34,ind_i);
  q_data(:,i) = mean(x_data(1:34,[ind_i-avg_range:ind_i+avg_range]),2);
  ind_i = find(t_vicon > t_sample(i),1);
%   torso_data(:,:,i) = torso_markers(1:3,:,ind_i);
%   hand_data(:,:,i) = hand_markers(1:3,:,ind_i);
  torso_data(:,:,i) = R'*mean(torso_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  hand_data(:,:,i) = R'*mean(hand_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  
  left_hand_data(:,:,i) = R'*mean(left_hand_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  pelvis_data(:,:,i) = R'*mean(pelvis_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  
  torso_obsc = find(sum(torso_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));
  hand_obsc = find(sum(hand_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));

  torso_data(:,torso_obsc,i) = NaN*torso_data(:,torso_obsc,i);
  hand_data(:,hand_obsc,i) = NaN*hand_data(:,hand_obsc,i);
end

%%
[dq, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = ...
  jointOffsetCalibration(r, q_data, joint_indices,torso_body,@(params) torsoMarkerPos_20131123(params,false), 6, torso_data, hand_body, @rightHandMarkerPos_20131123, 12, hand_data);
dq = unwrap([0;dq]);
dq = dq(2:end);
dq*180/pi

%%
[dq, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = ...
  jointOffsetCalibration(r, q_data, [],torso_body,@(params) torsoMarkerPos_20131123(params,true), 0, torso_data, hand_body, @noPointsFun, 0, zeros(3,0,length(t_sample)));
dq = unwrap([0;dq]);
dq = dq(2:end); 
dq*180/pi
fs_torsoonly = floating_states;
%%
q_fs = q_data;
q_fs(1:6,:) = fs_torsoonly;
[dq, body1_params, body2_params, floating_states, residuals, info, J, body1_resids, body2_resids] = ...
  jointOffsetCalibration(r, q_fs, joint_indices,torso_body,@(params) torsoMarkerPos_20131123(params,true), 0, torso_data, hand_body, @rightHandMarkerPos_20131123, 12, hand_data,struct('search_floating',false));
dq = unwrap([0;dq]);
dq = dq(2:end);
dq*180/pi

floating_states = fs_torsoonly;

%%
% I_unobs_hand = find(~any(hand_markers(4:4:end,:,:)));
% 
% for i=1:5,
%   for j=1:5,
%     dvec = hand_markers(1:3,i,I_unobs_hand) - hand_markers(1:3,j,I_unobs_hand);
%     d(i,j) = sqrt(mean(sum(dvec.*dvec)));
%   end
% end


%%
v = r.constructVisualizer;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');
j = 1;
q = q_data(:,j);
% q(setdiff(1:34,joint_indices),:) = 0*q(setdiff(1:34,joint_indices),:);
% q = q*0 
q(1:6) = floating_states(:,j);
% q(joint_indices) = q(joint_indices(randperm(6)));
% q(6) = pi/2;    
% q = q*0;
% b2 = [-.1 -.15 -.2 .1 -.1 .15]';
% b2 = body2_params;
% q(3) = 3;
% b2(2) = -.095 - .104;
q(joint_indices) = q(joint_indices) + dq;
% q(3) = 3;
% q = q*0;
% q(6) = pi/2;
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

for i=1:size(left_hand_markers,2),
  lcmgl.glColor3f(1,0,0); % red
  lcmgl.sphere(handpts(:,i),.01,20,20);
  lcmgl.glColor3f(0,0,1); % blue
  lcmgl.sphere(left_hand_data(:,i,j),.01,20,20);
end

for i=1:size(pelvis_markers,2),
  lcmgl.glColor3f(1,0,0); % red
%   lcmgl.sphere(handpts(:,i),.01,20,20);
  lcmgl.glColor3f(0,0,1); % blue
  lcmgl.sphere(pelvis_data(:,i,j),.01,20,20);
end
lcmgl.switchBuffers();

%%

% t_check = [3 16 20.7 31 56.9 64.5 88.3 100.9];
t_check = rand*max(t_vicon);
% t_check = [];
% t_check = 10;
qd_check = 1;
torso_check = 1;
hand_check = 1;
if length(t_check > 0) 
while max(max(abs(qd_check))) > .05 || any(any(isnan([torso_check(:);hand_check(:);q_check(:)])))
t_check = rand*max(t_vicon*.99);
% t_check = 42.9;
% t_check = [42.9 120 131.4 133.5 143.5 147.9 155.6];


clear q_check torso_check hand_check qd_check
for i=1:length(t_check),
  ind_i = find(t_x > t_check(i),1);
  
  q_check(:,i) = mean(x_data(1:34,[ind_i-avg_range:ind_i+avg_range]),2);
  qd_check(:,i) = mean(x_data(35:68,[ind_i-avg_range:ind_i+avg_range]),2);

  ind_i = find(t_vicon > t_check(i),1);
  %   torso_check(:,:,i) = torso_markers(1:3,:,ind_i);
  %   hand_check(:,:,i) = hand_markers(1:3,:,ind_i);
  torso_check(:,:,i) = mean(torso_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  hand_check(:,:,i) = mean(hand_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  
  torso_obsc = find(sum(torso_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));
  hand_obsc = find(sum(hand_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));
  
  torso_check(:,torso_obsc,i) = NaN*torso_check(:,torso_obsc,i);
  hand_check(:,hand_obsc,i) = NaN*hand_check(:,hand_obsc,i);  
end
q_check(joint_indices,:) = q_check(joint_indices,:) + repmat(dq,1,length(t_check));
q_check;
end

[~, ~, ~, floating_states_check, residual_check, info_check, J_check, body1_resids_check, body2_resids_check] = ...
  jointOffsetCalibration(r, q_check, [],torso_body,@(params) torsoMarkerPos_20131123(params,true), 0, torso_check, hand_body, @(params) rightHandMarkerPos_20131123(body2_params,true), 0, hand_check);


if length(t_check > 0)
v = r.constructVisualizer;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');
j = 1;
q = q_check(:,j);
% q(setdiff(1:34,joint_indices),:) = 0*q(setdiff(1:34,joint_indices),:);

q(1:6) = floating_states_check(:,j);
% q(2) = -.3;
% b2 = [-.1 -.15 -.2 .1 -.1 .15]';
b2 = body2_params;
% b2(2) = -.095;

q(joint_indices,1) = q(joint_indices,1);
v.draw(0,q);
kinsol = r.doKinematics(q);
handpts = r.forwardKin(kinsol,hand_body,handMarkerPos_newmarkers(b2));
torsopts = r.forwardKin(kinsol,torso_body,torsoMarkerPos_newmarkers(body1_params,true));

for i=1:size(torso_markers,2),
  lcmgl.glColor3f(1,0,0); % red
  lcmgl.sphere(torsopts(:,i),.01,20,20);
  lcmgl.glColor3f(0,0,1); % blue
  lcmgl.sphere(torso_check(:,i,j),.01,20,20);
end

for i=1:size(hand_markers,2),
  lcmgl.glColor3f(1,0,0); % red
  lcmgl.sphere(handpts(:,i),.01,20,20);
  lcmgl.glColor3f(0,0,1); % blue
  lcmgl.sphere(hand_check(:,i,j),.01,20,20);
end
lcmgl.switchBuffers();
end
sprintf('mean err (mm): %d',mean(sqrt(sum(body2_resids_check.*body2_resids_check)))*1000)
end

%% planar checking of pts [1 2 3 5]
for i=1:length(t_sample)
  v1 = hand_data(:,2,i) - hand_data(:,1,i);
  v2 = hand_data(:,3,i) - hand_data(:,1,i);
  n = cross(v1,v2);
  n = n/norm(n);
  d = hand_data(:,1,i)'*n;
  plane_err(i) = hand_data(:,5,i)'*n - d;
end