%%

data= dlmread('true_estimated_states.csv');



t = data(:,25);

Ts = mean(diff(t))*1E-9;

t = linspace(0,(t(end)-t(1))*Ts,length(t));

tpos = data(:,1:3);
tvel = data(:,4:6);
tE = data(:,7:9);
trate = data(:,10:12);

epos = data(:,13:15);
evel = data(:,16:18);
eE = data(:,19:21);
erate = data(:,22:24);

%% Plotting

close all

spa=4;
spb=1;
r2d = 180/pi;

subplot(spa,spb,1)

plot(t,tpos), hold on, grid on, xlabel('Sim time [s]'), title('positions')
plot(t,epos,'--')

subplot(spa,spb,2)
plot(t,tvel), hold on, grid on, title('velocities')
plot(t,evel,'--')

subplot(spa,spb,3)
plot(t,tE*r2d), hold on, grid on, title('Euler angles [deg]')
plot(t,eE*r2d,'--')

subplot(spa,spb,4)
plot(t,trate*r2d), hold on, grid on,title('rates [deg/s]')
plot(t,erate*r2d,'--')


figure(2)

subplot(spa,spb,1)

plot(t,tpos-epos), grid on, xlabel('Sim time [s]'), title('position errors')

subplot(spa,spb,2)
plot(t,tvel-evel), grid on, title('velocity errors')

subplot(spa,spb,3)
plot(t,(tE-eE)*r2d), grid on, title('Euler angle errors [deg]')

subplot(spa,spb,4)
plot(t,(trate-erate)*r2d), grid on,title('rate errors [deg/s]')


%% adjust plots

%plot at so many sigma
sigma_plot = 4;

[x,y] = ginput();

start = find(t>x(1),1); %start position in x
stop = length(t)-find(fliplr(t)<x(2),1); % stop position in x

for n=1:spa
    subplot(spa,spb,n);
    v=axis;
    
    y=get(findobj(get(gca,'Children'),'type','line'),'ydata');
    Ystd = [];
    for n_lin = 1:length(y)
         Ystd = [Ystd std(y{n_lin}(1,start:stop))];
         
    end
    v(3) = -sigma_plot*max(Ystd);
    v(4) = sigma_plot*max(Ystd);
    
    for n_lin = 1:length(y)
        if (min(y{n_lin}(1,start:stop)) < v(3))
            v(3) = min(y{n_lin}(1,start:stop))
        end
        if (max(y{n_lin}(1,start:stop)) > v(4))
            v(4) = max(y{n_lin}(1,start:stop))
        end
    end
    
    axis([x' v(3:4)]);
    
end



%% Sensor performance

gyro_std = std(erate)*180/pi




