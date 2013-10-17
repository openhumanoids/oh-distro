%NOTEST
if ~exist('r')
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
end
% logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-11.00_arm_massage');
logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-16.02');

[t_x,x_data,t_u,u_data,t_vicon,vicon_data,state_frame,input_frame,t_extra,extra_data,vicon_data_struct] = parseAtlasViconLog(r,logfile);
% t_extra = t_extra(1:end-1);
% extra_data = extra_data(:,1:end-1);

t_u = t_u - t_x(1);
t_extra = t_extra - t_x(1);
t_x = t_x - t_x(1);

%%
est_state_joint_indices = [10:14 21 22:26 33];
extra_joint_indicies = 16 + (1:12);

for i=1:12,
  extra_data(extra_joint_indicies(i),:) = unwrap(extra_data(extra_joint_indicies(i),:));% - extra_data(extra_joint_indicies(i),1) + x_data(est_state_joint_indices(i),1);
end

figure(1)
for i=1:6,
  subplot(3,2,i)
  plot(extra_data(extra_joint_indicies(i),:)*180/pi, (x_data(est_state_joint_indices(i),:) - extra_data(extra_joint_indicies(i),:))*180/pi);
  xlabel('Encoder Measurement (deg)')
  ylabel('Pot - Encoder (deg)')
  title(strrep(r.getStateFrame.coordinates{est_state_joint_indices(i)},'_',' '))
end

figure(2)
for i=7:12,
  subplot(3,2,i-6)
  plot(extra_data(extra_joint_indicies(i),:)*180/pi, (x_data(est_state_joint_indices(i),:) - extra_data(extra_joint_indicies(i),:))*180/pi);
  xlabel('Encoder Measurement (deg)')
  ylabel('Pot - Encoder (deg)')
  title(strrep(r.getStateFrame.coordinates{est_state_joint_indices(i)},'_',' '))
end

%%
for i=1:12,
  L(i,:) = polyfit(extra_data(extra_joint_indicies(i),:), x_data(est_state_joint_indices(i),:), 1);
end
%% index by encoder measurement
res = .01;
for i=1:12,
  enc_min = min(extra_data(extra_joint_indicies(i),:));
  enc_max = max(extra_data(extra_joint_indicies(i),:));
  enc_i = [];
  pot_i = [];
  theta = enc_min;
  while theta <= enc_max
    I = find(extra_data(extra_joint_indicies(i),:) >= theta & extra_data(extra_joint_indicies(i),:) < theta + res);
    theta = theta + res;
%     pot_i = [pot_i;mean(x_data(est_state_joint_indices(i),I))];
    pot_i = [pot_i;(min(x_data(est_state_joint_indices(i),I)) + max(x_data(est_state_joint_indices(i),I)))/2];
    enc_i = [enc_i;theta];
  end
  linear_fit(i,:) = polyfit(enc_i, pot_i, 1);
end

%%
figure(3)
for i=1:6,
  subplot(3,2,i)
  plot(extra_data(extra_joint_indicies(i),:)*180/pi, (x_data(est_state_joint_indices(i),:) - extra_data(extra_joint_indicies(i),:))*180/pi, extra_data(extra_joint_indicies(i),:)*180/pi, 180/pi*(polyval(linear_fit(i,:), extra_data(extra_joint_indicies(i),:))-extra_data(extra_joint_indicies(i),:)))
  xlabel('Encoder Measurement (deg)')
  ylabel('Pot - Encoder (deg)')
  title(strrep(r.getStateFrame.coordinates{est_state_joint_indices(i)},'_',' '))
end

figure(4)
for i=7:12,
  subplot(3,2,i-6)
  plot(extra_data(extra_joint_indicies(i),:)*180/pi, (x_data(est_state_joint_indices(i),:) - extra_data(extra_joint_indicies(i),:))*180/pi, extra_data(extra_joint_indicies(i),:)*180/pi, 180/pi*(polyval(linear_fit(i,:), extra_data(extra_joint_indicies(i),:))-extra_data(extra_joint_indicies(i),:)))
  xlabel('Encoder Measurement (deg)')
  ylabel('Pot - Encoder (deg)')
  title(strrep(r.getStateFrame.coordinates{est_state_joint_indices(i)},'_',' '))
end