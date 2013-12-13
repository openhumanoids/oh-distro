close all
clear all

data = load('orientation_drift.txt')

subplot(2,1,1)
plot(data(:,[3]))
title('linear velocity, 10sec')

subplot(2,1,2)
plot(data(:,[4]))
title('yaw change, 10sec')