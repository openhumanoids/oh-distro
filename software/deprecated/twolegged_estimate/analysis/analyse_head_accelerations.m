


%% Head imu analysis


t = himu(:,1)/1E6;

acc = himu(:,2:4);

plot(t,acc)
grid on
ylabel('Acceleration [m/{s^2}]')
xlabel('Sim time [s]')

[x,y] = ginput();

v = axis;

axis([x(1) x(2) v(3) v(4)])
