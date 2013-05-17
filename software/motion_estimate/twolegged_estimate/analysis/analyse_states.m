%%

legkinvels = size(data,2)>91;



clear

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

% check if kinematic foot velocities are available
legkinvels = size(data,2)>91;

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


%joint commands

offset = 31;

ce_l_uhz = data(:,(offset+4));
ce_l_mhx = data(:,(offset+5));
ce_l_lhy = data(:,(offset+6));
ce_l_kny = data(:,(offset+7));
ce_l_uay = data(:,(offset+8));
ce_l_lax = data(:,(offset+9));
ce_r_uhz = data(:,(offset+10));
ce_r_mhx = data(:,(offset+11));
ce_r_lhy = data(:,(offset+12));
ce_r_kny = data(:,(offset+13));
ce_r_uay = data(:,(offset+14));
ce_r_lax = data(:,(offset+15));

% offset = offset+28;
offset = 59;

jp_l_uhz = data(:,(offset+4));
jp_l_mhx = data(:,(offset+5));
jp_l_lhy = data(:,(offset+6));
jp_l_kny = data(:,(offset+7));
jp_l_uay = data(:,(offset+8));
jp_l_lax = data(:,(offset+9));
jp_r_uhz = data(:,(offset+10));
jp_r_mhx = data(:,(offset+11));
jp_r_lhy = data(:,(offset+12));
jp_r_kny = data(:,(offset+13));
jp_r_uay = data(:,(offset+14));
jp_r_lax = data(:,(offset+15));

% offset = offset+16;
offset = 75;

% me_l_uhz = data(:,(offset+4));
% me_l_mhx = data(:,(offset+5));
% me_l_lhy = data(:,(offset+6));
% 
% me_l_lax = data(:,(offset+9));
% me_r_lax = data(:,(offset+15));

me_l_uhz = data(:,(offset+4));
me_l_mhx = data(:,(offset+5));
me_l_lhy = data(:,(offset+6));
me_l_kny = data(:,(offset+7));
me_l_uay = data(:,(offset+8));
me_l_lax = data(:,(offset+9));
me_r_uhz = data(:,(offset+10));
me_r_mhx = data(:,(offset+11));
me_r_lhy = data(:,(offset+12));
me_r_kny = data(:,(offset+13));
me_r_uay = data(:,(offset+14));
me_r_lax = data(:,(offset+15));

% 	  ss << joint_commands[i] << ", "; //31-58
% 		ss << joint_positions[i] << ", "; //59-74
% 	   ss << measured_joint_effort[i] << ", ";//75-90

% if (legkinvels)
    %foot velocities are available 

    offset = 91;
    
    l_foot_velx = data(:,(offset));
    l_foot_vely = data(:,(offset+1));
    l_foot_velz = data(:,(offset+2));
    
    r_foot_velx = data(:,(offset+3));
    r_foot_vely = data(:,(offset+4));
    r_foot_velz = data(:,(offset+5));
    
    l_foot_ratex = data(:,(offset+6));
    l_foot_ratey = data(:,(offset+7));
    l_foot_ratez = data(:,(offset+8));
    
    r_foot_ratex = data(:,(offset+9));
    r_foot_ratey = data(:,(offset+10));
    r_foot_ratez = data(:,(offset+11));
    
    
    jv_l_uhz = [0;diff(jp_l_uhz)*1E3];
    jv_l_mhx = [0;diff(jp_l_mhx)*1E3];
    jv_l_lhy = [0;diff(jp_l_lhy)*1E3];
    jv_l_kny = [0;diff(jp_l_kny)*1E3];
    jv_l_uay = [0;diff(jp_l_uay)*1E3];
    jv_l_lax = [0;diff(jp_l_lax)*1E3];
    jv_r_uhz = [0;diff(jp_r_uhz)*1E3];
    jv_r_mhx = [0;diff(jp_r_mhx)*1E3];
    jv_r_lhy = [0;diff(jp_r_lhy)*1E3];
    jv_r_kny = [0;diff(jp_r_kny)*1E3];
    jv_r_uay = [0;diff(jp_r_uay)*1E3];
    jv_r_lax = [0;diff(jp_r_lax)*1E3];
    
    
% end


%% Plotting

% close all

spa=4;
spb=1;
r2d = 180/pi;

figure(1); clf


subplot(spa,spb,1)

plot(t,tpos), hold on, grid on, xlabel('Sim time [s]'), title('positions'),
plot(t,epos,'--')
plot(t,left_con,':m','linewidth',2)
plot(t,right_con,':c','linewidth',2)
legend('true x','true y','true z','est x','est y','est z','left c','righ c')

subplot(spa,spb,2)
plot(t,tvel), hold on, grid on, title('velocities')
plot(t,evel,'--')

subplot(spa,spb,3)
plot(t,tE*r2d), hold on, grid on, title('Euler angles [deg]')
plot(t,eE*r2d,'--')

subplot(spa,spb,4)
plot(t,trate*r2d), hold on, grid on,title('rates [deg/s]')
plot(t,erate*r2d,'--')


if (false)
    figure

    subplot(spa,spb,1)

    plot(t,tpos-epos), grid on, xlabel('Sim time [s]'), title('position errors')

    subplot(spa,spb,2)
    plot(t,tvel-evel), grid on, title('velocity errors')

    subplot(spa,spb,3)
    plot(t,(tE-eE)*r2d), grid on, title('Euler angle errors [deg]')

    subplot(spa,spb,4)
    plot(t,(trate-erate)*r2d), grid on,title('rate errors [deg/s]')

end



% knees and ankles

figure(2); clf


spa = 4;
spb = 1;

scale = 200;
subplot(spa,spb,1),
plot(t,evel)
hold on
plot(t,tvel,'--','linewidth',2)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
legend({'est vel x', 'est vel y', 'est vel z', 'true vel x', 'true vel y', 'true vel z', 'left c','righ c'})
title('States')

subplot(spa,spb,2),
plot(t,[ce_l_lax, ce_r_lax],'--')
hold on
plot(t,[me_l_lax, me_r_lax, jp_l_lax*scale, jp_r_lax*scale])
grid on
legend({'ce l lax', 'ce r lax', 'me l lax', 'me r lax', 'jp l lax', 'jp r lax'})
title('Ankle X')

subplot(spa,spb,3),
plot(t,[ce_l_uay, ce_r_uay],'--')
hold on
plot(t,[me_l_uay, me_r_uay, jp_l_uay*scale, jp_r_uay*scale])
legend({'ce l uay', 'ce r uay', 'me l uay', 'me r uay', 'jp l uay', 'jp r uay'})
grid on
title('Ankle Y')

subplot(spa,spb,4)
plot(t,[ce_l_kny, ce_r_kny],'--')
hold on
plot(t,[me_l_kny, me_r_kny, jp_l_kny*scale, jp_r_kny*scale])
legend({'ce l kny', 'ce r kny', 'me l kny', 'me r kny', 'jp l kny', 'jp r kny'})
grid on
title('Knee Y')


% hips

figure(3); clf


spa = 4;
spb = 1;

scale = 200;
subplot(spa,spb,1),
plot(t,evel)
hold on
plot(t,tvel,'--','linewidth',2)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
legend({'est vel x', 'est vel y', 'est vel z', 'left c','righ c'})
title('States')

subplot(spa,spb,2),
plot(t,[ce_l_mhx, ce_r_mhx],'--')
hold on
plot(t,[me_l_mhx, me_r_mhx, jp_l_mhx*scale, jp_r_mhx*scale])
grid on
legend({'ce l mhx', 'ce r mhx', 'me l mhx', 'me r mhx', 'jp l mhx', 'jp r mhx'})
title('Hips X')

subplot(spa,spb,3),
plot(t,[ce_l_lhy, ce_r_lhy],'--')
hold on
plot(t,[me_l_lhy, me_r_lhy, jp_l_lhy*scale, jp_r_lhy*scale])
legend({'ce l lhy', 'ce l lhy', 'me l lhy', 'me r lhy', 'jp l lhy', 'jp r lhy'})
grid on
title('Hips Y')

subplot(spa,spb,4)
plot(t,[ce_l_uhz, ce_r_uhz],'--')
hold on
plot(t,[me_l_uhz, me_r_uhz, jp_l_uhz*scale, jp_r_uhz*scale])
legend({'ce l uhz', 'ce l uhz', 'me l uhz', 'me r uhz', 'jp l uhz', 'jp r uhz'})
grid on
title('Hips Z')

% If leg kinematic velcoties are available
if (legkinvels)
    figure(4); clf
    
    spa = 4;
    spb = 1;
    
    subplot(spa,spb,1),
    plot(t,[l_foot_velx, l_foot_vely, l_foot_velz]);
    grid on
    title('pelvis to left foot linear velocity')
    
    subplot(spa,spb,2),
    plot(t,[r_foot_velx, r_foot_vely, r_foot_velz]);
    grid on
    title('pelvis to right foot linear velocity')
    
    subplot(spa,spb,3),
    plot(t,[l_foot_ratex, l_foot_ratey, l_foot_ratez]);
    grid on
    title('pelvis to left foot rate')
    
    subplot(spa,spb,4),
    plot(t,[r_foot_ratex, r_foot_ratey, r_foot_ratez]);
    grid on
    title('pelvis to right foot rate')
    
    
    figure(5); clf


    spa = 4;
    spb = 1;

    scale = 200;
    subplot(spa,spb,1),
    plot(t,[jv_l_mhx, jv_l_lhy, jv_l_uhz])
    grid on
    legend({'jv l mhx', 'jv l lhy', 'jv l uhz'})
    title('Left Hip Joint Velocities')

    subplot(spa,spb,2),
    plot(t,[jv_r_mhx, jv_r_lhy, jv_r_uhz])
    grid on
    legend({'jv r mhx', 'jv r lhy', 'jv r uhz'})
    title('Right Hip Joint Velocities')

    subplot(spa,spb,3),
    plot(t,[jv_l_kny, jv_l_uay, jv_l_lax])
    grid on
    legend({'jv l kny', 'jv l uay', 'jv l lax'})
    title('Left knee and ankle Joint Velocities')

    subplot(spa,spb,4)
    plot(t,[jv_r_kny, jv_r_uay, jv_r_lax])
    grid on
    legend({'jv r kny', 'jv r uay', 'jv r lax'})
    title('Right knee and ankle Joint Velocities')
    
end




figure(1)

%% adjust all plots

%plot at so many sigma
sigma_plot = 4;

figs = findobj('Type','figure');

% figure(1)

    [x,y] = ginput();
    
    start = find(t>x(1),1); %start position in x
    stop = length(t)-find(fliplr(t)<x(2),1); % stop position in x
    spa=4;
    spb=1;

for k=1:length(figs)
    figure(figs(k));

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
                v(3) = min(y{n_lin}(1,start:stop))*1.1;
            end
            if (max(y{n_lin}(1,start:stop)) > v(4))
                v(4) = max(y{n_lin}(1,start:stop))*1.1;
            end
        end

        axis([x' v(3:4)]);

    end
end

%% adjust plots for 4, 1
if (false)
    %plot at so many sigma
    sigma_plot = 4;

    spa=4;
    spb=1;

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
                v(3) = min(y{n_lin}(1,start:stop))*1.1;
            end
            if (max(y{n_lin}(1,start:stop)) > v(4))
                v(4) = max(y{n_lin}(1,start:stop))*1.1;
            end
        end

        axis([x' v(3:4)]);

    end
end

