%NOTEST 

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options); 
seframe = AtlasStateAndEffort(r);
uframe = AtlasPosVelTorqueRef(r);
% [t_x,x_data,t_u,u_data] = parseAtlasLog(r,'/home/drc/r_arm_shx.log');
[t_x,x_data,t_u,u_data] = parseAtlasLog(r,'/home/drc/r_arm_elx.log');
% [t_x,x_data,t_u,u_data] = parseAtlasLog(r,'/home/drc/r_arm_mwx.log');
% [t_x,x_data,t_u,u_data] = parseAtlasLog(r,'/home/drc/r_arm_uwy.log');
% [t_x,x_data,t_u,u_data] = parseAtlasLog(r,'/home/drc/r_arm_ely.log');
command_0_ind = find(diff(t_u) == 0);
t_u(command_0_ind + 1) = t_u(command_0_ind + 1) + 1e-6;

t_u = t_u - t_x(1);
t_x = t_x - t_x(1);

u_interp = zeros(size(u_data,1),length(t_x));
for i=1:size(u_data,1);
    u_interp(i,:) = interp1(t_u,u_data(i,:),t_x,'nearest','extrap');
end

%%
% j_name = 'r_arm_mwx';
% q0 = 0;
% threshold = .05;
% t_ramp_end = 210;

j_name = 'r_arm_elx';
q0 = -pi/2;
threshold = .02;
t_ramp_end = 255;

% j_name = 'r_arm_uwy';
% q0 = pi/2;
% threshold = .1;
% t_ramp_end = 200;

% j_name = 'r_arm_ely';
% q0 = pi/2;
% threshold = .05;
% t_ramp_end = 250;

% j_name = 'r_arm_shx';
% q0 = pi/2;
% threshold = .02;
% t_ramp_end = 250;
% 
% j_name = 'r_arm_shy';
% q0 = pi/2;
% threshold = .1;
% t_ramp_end = 200;


plot_joint.q_ind = getStringIndices(seframe.coordinates,j_name);
plot_joint.qd_ind = getStringIndices(seframe.coordinates,strcat(j_name,'_dot'));
plot_joint.u_ind = getStringIndices(seframe.coordinates,strcat(j_name,'_effort'));
plot_joint.uc_ind = getStringIndices(uframe.coordinates,strcat(j_name,'_effort'));


% plot_joint = uwy_data;
% close all

%isolate positions near the nominal position (q0) and where the robot is on
%torque control mode(here identified by commanded torque != 0)
I = find(abs(x_data(plot_joint.q_ind,:) - q0) < threshold & u_interp(plot_joint.uc_ind,:) ~= 0);
I_ramp_end = find(I > find(t_x > t_ramp_end,1),1);

figure(1)
subplot(3,1,1)
plot(t_x,x_data(plot_joint.q_ind,:))
ylabel('Position')
xlabel('Time')
subplot(3,1,2)
plot(t_x,x_data(plot_joint.qd_ind,:))
ylabel('Velocity')
xlabel('Time')
subplot(3,1,3)
plot(t_x,x_data(plot_joint.u_ind,:),t_u,u_data(plot_joint.uc_ind,:))
ylabel('Torque')
xlabel('Time')

figure(2)
subplot(3,1,1)
plot(t_x(I),x_data(plot_joint.q_ind,I),'*')
ylabel('Position')
xlabel('Time')
subplot(3,1,2)
plot(t_x(I),x_data(plot_joint.qd_ind,I),'*')
ylabel('Velocity')
xlabel('Time')
subplot(3,1,3)
plot(t_x(I),x_data(plot_joint.u_ind,I),'*')
ylabel('Torque')
xlabel('Time')

figure(3)
hold off
plot(x_data(plot_joint.qd_ind,I),x_data(plot_joint.u_ind,I),'*')
hold on
plot(x_data(plot_joint.qd_ind,I(1:I_ramp_end)),x_data(plot_joint.u_ind,I(1:I_ramp_end)),'r*')


%%
valve = .0025 * u_interp(plot_joint.uc_ind,:) + .125 * (u_interp(plot_joint.uc_ind,:) - x_data(plot_joint.u_ind,:));
figure(4)
subplot(3,1,1)
plot(t_x,valve)
xlim([285 330])
subplot(3,1,2)
plot(t_x,x_data(plot_joint.qd_ind,:))
xlim([285 330])
subplot(3,1,3)
plot(t_x,x_data(plot_joint.u_ind,:))
xlim([285 330])

% linear fit
I_fit = find(t_x < 330 & t_x > 285);
Y = valve(I_fit)';
X = [x_data(plot_joint.qd_ind,I_fit); x_data(plot_joint.u_ind,I_fit)]';
gains = pinv(X)*Y

figure(5)
plot(t_x(I_fit),valve(I_fit),t_x(I_fit),gains(1) * x_data(plot_joint.qd_ind,I_fit) + gains(2) * x_data(plot_joint.u_ind,I_fit))


%elx: gains .36, 0