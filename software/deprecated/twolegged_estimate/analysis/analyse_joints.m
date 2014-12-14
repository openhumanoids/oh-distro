%% Analyze joint positions recorded in leg odometry model

clear data_j;
data_j = dlmread('joint_data.csv');








%% Plotting of the joint data

% close all

a = 4;
b = 1;

subplot(a,b,1)
plot(data_j(:,1:4))
grid on
legend({num2str((1:4)')})

subplot(a,b,2)
plot(data_j(:,5:8))
grid on

subplot(a,b,3)
plot(data_j(:,9:12))
grid on

subplot(a,b,4)
plot(data_j(:,13:16))
grid on
