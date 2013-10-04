%NOTEST
if ~exist('r')
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
end
logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-02-vicon-test');
[t_x,x_data,t_u,u_data,t_vicon,vicon_data,state_frame,input_frame] = parseAtlasViconLog(r,logfile,10);
t_u = t_u - t_x(1);
t_vicon = t_vicon - t_x(1);
t_x = t_x - t_x(1);

joint_indices = [22:26 33];
% joint_indices = [23:26 33];
% joint_indices = [];

% Sample times
% todo, maybe filter the data around them?

% t_sample = [21.6 34.5 45.2 62 71 82 93.9];  %something is wrong with the
% third one here
% t_sample = [21.6 34.5 62 71 82 93.9];
t_sample = [4.5 21.6 28.1 34.5 62 71 82 93.9];

torso_markers = reshape(vicon_data(1:20,:),4,5,[]);
hand_markers = reshape(vicon_data(21:end,:),4,5,[]);

torso_markers(1:3,:,:) = torso_markers(1:3,:,:)/1e3;
hand_markers(1:3,:,:) = hand_markers(1:3,:,:)/1e3;

%%
hand_markers = hand_markers - reshape(repmat([mean(mean(torso_markers(1:3,:,:),2),3);0],size(hand_markers,2)*size(hand_markers,3),1),4,size(hand_markers,2),[]);
torso_markers = torso_markers - reshape(repmat([mean(mean(torso_markers(1:3,:,:),2),3);0],size(torso_markers,2)*size(torso_markers,3),1),4,size(torso_markers,2),[]);

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

%%
torso_body = 5;
hand_body = 29;

for i=1:length(t_sample),
  q_data(:,i) = x_data(1:34,find(t_x > t_sample(i),1));
  torso_data(:,:,i) = torso_markers(1:3,:,find(t_vicon > t_sample(i),1));
  hand_data(:,:,i) = hand_markers(1:3,:,find(t_vicon > t_sample(i),1));
end

%%
[dq, body1_params, body2_params, floating_states, residuals, info, J] = ...
  jointOffsetCalibration(r, q_data, joint_indices,torso_body,@torsoMarkerPos, 9, torso_data, hand_body, @handMarkerPos, 4, hand_data);

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
q(setdiff(1:34,joint_indices),:) = 0*q(setdiff(1:34,joint_indices),:);

q(1:6) = floating_states(:,j);

q(joint_indices) = q(joint_indices) + dq;
v.draw(0,q);
kinsol = r.doKinematics(q);
handpts = r.forwardKin(kinsol,hand_body,handMarkerPos(body2_params));
torsopts = r.forwardKin(kinsol,torso_body,torsoMarkerPos(body1_params));

for i=1:5,
  lcmgl.glColor3f(1,0,0); % red
  lcmgl.sphere(torsopts(:,i),.01,20,20);
  lcmgl.sphere(handpts(:,i),.01,20,20);
  lcmgl.glColor3f(0,0,1); % blue
  lcmgl.sphere(torso_data(:,i,j),.01,20,20);
  lcmgl.sphere(hand_data(:,i,j),.01,20,20);
end
lcmgl.switchBuffers();