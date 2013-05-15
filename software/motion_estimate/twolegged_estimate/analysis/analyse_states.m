%%

clear data
clear t
clear Ts
clear tpos
clear epos
clear tvel
clear evel
clear eE
clear tE
clear trate
clear erate

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

left_con = data(:,29);
right_con = data(:,30);

%%

% Left foot

%joint commands

offset = 31;

ce_l_uhz = data(:,(offset+4));
ce_l_mhx = data(:,(offset+5));
ce_l_lhy = data(:,(offset+6));

%a
ce_l_lax = data(:,(offset+9));
ce_r_lax = data(:,(offset+15));

% offset = offset+28;
offset = 59;

jp_l_uhz = data(:,(offset+4));
jp_l_mhx = data(:,(offset+5));
jp_l_lhy = data(:,(offset+6));

jp_l_lax = data(:,(offset+9));
jp_r_lax = data(:,(offset+15));

% offset = offset+16;
offset = 75;

me_l_uhz = data(:,(offset+4));
me_l_mhx = data(:,(offset+5));
me_l_lhy = data(:,(offset+6));

me_l_lax = data(:,(offset+9));
me_r_lax = data(:,(offset+15));

% right foot

offset = 31+3;

ce_r_uhz = data(:,(offset+4));
ce_r_mhx = data(:,(offset+5));
ce_r_lhy = data(:,(offset+6));

% offset = offset+28;
offset = 59+3;

jp_r_uhz = data(:,(offset+4));
jp_r_mhx = data(:,(offset+5));
jp_r_lhy = data(:,(offset+6));


% offset = offset+16;
offset = 75+3;

me_r_uhz = data(:,(offset+4));
me_r_mhx = data(:,(offset+5));
me_r_lhy = data(:,(offset+6));







% 	  ss << joint_commands[i] << ", "; //31-58
% 		ss << joint_positions[i] << ", "; //59-74
% 	   ss << measured_joint_effort[i] << ", ";//75-90

%% Plotting

close all

spa=4;
spb=1;
r2d = 180/pi;

subplot(spa,spb,1)

plot(t,tpos), hold on, grid on, xlabel('Sim time [s]'), title('positions'),
plot(t,epos,'--')
plot(t,left_con,':m','linewidth',2)
plot(t,right_con,':c','linewidth',2)
legend('true x','true y','true z','est x','est y','est z','left_c','righ_c')

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
            v(3) = min(y{n_lin}(1,start:stop));
        end
        if (max(y{n_lin}(1,start:stop)) > v(4))
            v(4) = max(y{n_lin}(1,start:stop));
        end
    end
    
    axis([x' v(3:4)]);
    
end



%% Sensor performance

gyro_std = std(erate)*180/pi



%% Joint traces

figure(3)

i_me = cumsum(me_l_uhz*1E-3);

subplot(211),plot(t,[ce_l_uhz, me_l_uhz, 100*jp_l_uhz]),grid on,legend({'ce l uhz';'me l uhz'; 'jp l uhz'})
subplot(212),plot(t,[ce_r_uhz, me_r_uhz, 100*jp_r_uhz]),grid on,legend({'ce r uhz';'me r uhz'; 'jp r uhz'})



figure(4)

subplot(211),plot(t,[ce_l_lhy, me_l_lhy, 100*jp_l_lhy]),grid on,legend({'ce l lhy';'me l lhy'; 'jp l lhy'})
subplot(212),plot(t,[ce_r_lhy, me_r_lhy, 100*jp_r_lhy]),grid on,legend({'ce r lhy';'me r lhy'; 'jp r lhy'})

figure(5)

subplot(211),plot(t,[ce_l_mhx, me_l_mhx, 100*jp_l_mhx]),grid on,legend({'ce l mhx';'me l mhx'; 'jp l mhx'})
subplot(212),plot(t,[ce_r_mhx, me_r_mhx, 100*jp_r_mhx]),grid on,legend({'ce r mhx';'me r mhx'; 'jp r mhx'})

%%

figure(6)

scale = 200;
subplot(211),
plot(t,[me_l_mhx, me_r_mhx],'--')
hold on
plot(t,[ce_l_mhx, ce_r_mhx, scale*epos(:,2), scale*tpos(:,2), scale*evel(:,2)/2, scale*tvel(:,2)/2])
grid on
legend({'me l mhx', 'me r mhx', 'ce l mhx', 'ce r mhx', 'est pos y', 'true pos y', 'est vel y', 'true vel y'})

subplot(212),
plot(t,[jp_l_lax, jp_r_lax, ce_l_lax, ce_r_lax])
legend({ 'jp l lax', 'jp r lax', 'ce l lax', 'ce r lax'})
grid on







