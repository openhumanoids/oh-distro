function [] = plotstates(statedata)

figure(20);clf

spa = 4;
spb = 1;





subplot(spa,spb,3)
plot(statedata.t,tvel-imu_V)
grid on
title('INS velocity errors')
grid on