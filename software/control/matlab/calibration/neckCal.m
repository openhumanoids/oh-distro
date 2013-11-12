%NOTEST
if ~exist('r')
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
end
logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-09-12-30_neck_vicon_calibration');


[t_x,x_data,t_u,u_data,t_vicon,vicon_data,state_frame,input_frame,t_extra,extra_data] = parseAtlasViconLog(r,logfile,14);
t_u = t_u - t_x(1);
t_vicon = t_vicon - t_x(1) - .56;
t_extra = t_extra - t_x(1);
t_x = t_x - t_x(1);

neck_ind = getStringIndices(r.getStateFrame.coordinates,'neck_ay');

torso_body = 5;
head_body = 30;

torso_markers = reshape(vicon_data(21:40,:),4,5,length(t_vicon));
hand_markers = reshape(vicon_data(1:20,:),4,5,length(t_vicon));
head_markers = reshape(vicon_data(41:end,:),4,4,length(t_vicon));

torso_markers(1:3,:,:) = torso_markers(1:3,:,:)/1e3;
hand_markers(1:3,:,:) = hand_markers(1:3,:,:)/1e3;
head_markers(1:3,:,:) = head_markers(1:3,:,:)/1e3;


%% use t=0 to find positions of markers
[~, ~, head_params, floating_states, residuals, info, J, body1_resids, body2_resids] = ...
  jointOffsetCalibration(r, x_data(1:34,1), [],torso_body,@(param) torsoMarkerPos(param,true), 0, torso_markers(1:3,:,1), head_body, @headMarkerPos, 12, head_markers(1:3,:,1));

%%
clear q_data torso_data hand_data head_data
% t_sample = linspace(120,180,20);
t_sample = [125 131.9 135.8 139.9 142.5 145 147.8 152.4 154 155.8 158.1 162.7 167.9 30 79.2 0];
t_sample = [t_sample linspace(125,175,100)];
t_sample = linspace(127,165,50);
avg_range = 0;
for i=1:length(t_sample),
  ind_i = find(t_x > t_sample(i),1);
%   q_data(:,i) = x_data(1:34,ind_i);
  q_data(:,i) = mean(x_data(1:34,[ind_i-avg_range:ind_i+avg_range]),2);
  ind_i = find(t_vicon > t_sample(i),1);
%   torso_data(:,:,i) = torso_markers(1:3,:,ind_i);
%   hand_data(:,:,i) = hand_markers(1:3,:,ind_i);
  torso_data(:,:,i) = mean(torso_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  hand_data(:,:,i) = mean(hand_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  head_data(:,:,i) = mean(head_markers(1:3,:,[ind_i-avg_range:ind_i+avg_range]),3);
  
  torso_obsc = find(sum(torso_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));
  hand_obsc = find(sum(hand_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));
  head_obsc = find(sum(head_markers(4,:,[ind_i-avg_range:ind_i+avg_range]),3));

  torso_data(:,torso_obsc,i) = NaN*torso_data(:,torso_obsc,i);
  hand_data(:,hand_obsc,i) = NaN*hand_data(:,hand_obsc,i);
  head_data(:,head_obsc,i) = NaN*head_data(:,head_obsc,i);
end

dq = zeros(length(t_sample),1);
head_resids = zeros(3,4,length(t_sample));
for i=1:length(t_sample),
[dq(i), ~, ~, floating_states, residuals, info, J, body1_resids, head_resids(:,:,i)] = ...
  jointOffsetCalibration(r, q_data(:,i), neck_ind,torso_body,@(param) torsoMarkerPos(param,true), 0, torso_data(:,:,i), head_body, @(param) headMarkerPos(head_params,true), 0, head_data(:,:,i));
end

[t_sample' reshape(sqrt(mean(sum(head_resids.*head_resids))),[],1)*1e3]

%%
plot(q_data(neck_ind,:)*180/pi,dq*180/pi,'*')

%% fit a polynomial model
deg = 4;

I=find(q_data(neck_ind,:)*180/pi >= -inf);
coefs = polyfit(q_data(neck_ind,I)',dq(I),deg)
plot(q_data(neck_ind,:)*180/pi,dq*180/pi,'*',180/pi*sort(q_data(neck_ind,I)),180/pi*polyval(coefs,sort(q_data(neck_ind,I))))
