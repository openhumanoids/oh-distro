function main()
close all
clear all

data = load('orientation_drift_drill_13dec2013.txt')
plotdata(data,'drill 13dec')

data = load('orientation_drift_debris_10dec2013.txt')
plotdata(data,'debris 10dec')


data = load('orientation_drift_debris_12dec2013_09_06.txt')
plotdata(data,'debris 12dec')

function plotdata(data,name)
figure 

subplot(2,1,1)
plot(data(:,[3]))
title('linear velocity, 1sec')

subplot(2,1,2)
plot(data(:,[4]))
title(['yaw change, 1sec ' name ])

