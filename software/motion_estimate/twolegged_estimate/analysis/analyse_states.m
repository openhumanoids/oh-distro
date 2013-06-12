%% You will need this path to access specific matlab scripts

addpath('~/drc/software/motion_estimate/twolegged_estimate/analysis')

%% change current directory to

cd '~/drc/software/config'

%% change current directory to

cd '~/drc/software/motion_estimate/twolegged_estimate'

%% Load the csv into matlab workspace
clc
vars = csv_to_mat('true_estimated_states.csv','');
field_names = fieldnames(vars);
for i = 1:numel(field_names)
    eval([field_names{i}, '=vars.', field_names{i}, ';']);
end


%% Plotting

q = [1/sqrt(2) 0 0 -1/sqrt(2)]';
C = q2C(q);


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
plot(t,[me_l_lax, me_r_lax, jp_l_lax*scale, jp_r_lax*scale, 0.5+fjp_l_lax*scale, 0.5+fjp_r_lax*scale ])
grid on
legend({'ce l lax', 'ce r lax', 'me l lax', 'me r lax', 'jp l lax', 'jp r lax', 'fjp l lax', 'fjp r lax'})
title('Ankle X')

subplot(spa,spb,3),
plot(t,[ce_l_uay, ce_r_uay],'--')
hold on
plot(t,[me_l_uay, me_r_uay, jp_l_uay*scale, jp_r_uay*scale, 0.5+fjp_l_uay*scale, 0.5+fjp_r_uay*scale])
legend({'ce l uay', 'ce r uay', 'me l uay', 'me r uay', 'jp l uay', 'jp r uay', 'fjp l uay', 'fjp r uay'})
grid on
title('Ankle Y')

subplot(spa,spb,4)
plot(t,[ce_l_kny, ce_r_kny],'--')
hold on
plot(t,[me_l_kny, me_r_kny, jp_l_kny*scale, jp_r_kny*scale, 0.5+fjp_l_kny*scale, 0.5+fjp_r_kny*scale])
legend({'ce l kny', 'ce r kny', 'me l kny', 'me r kny', 'jp l kny', 'jp r kny', 'fjp l kny', 'fjp r kny'})
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


figure(6); clf

subplot(spa,spb,1)
plot(t,evel)
hold on
plot(t,tvel,'--','linewidth',2)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
legend({'est vel x', 'est vel y', 'est vel z', 'left c','righ c'})
title('States')
grid on
title('Raw Vel');

subplot(spa,spb,2)
plot(t, stageA)
hold on
plot(t, stageB,'--','linewidth',2)
grid on
title('Stage A');

subplot(spa,spb,3)
plot(t, stageB)
grid on
title('Stage B');

subplot(spa,spb,4)
plot(t, stageC)
grid on
title('Stage C');




figure(7);clf

subplot(spa,spb,1)
plot(t,imu_P)
hold on
plot(t,tpos,'--','linewidth',2)
plot(t,epos,':','linewidth',2)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
title('States')
grid on

subplot(spa,spb,2)
plot(t,imu_V)
hold on
plot(t,leg_V,'--','linewidth',2)
 plot(t,tvel,':','linewidth',3)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
title('Vel States - Leg Odo')
grid on

subplot(spa,spb,3)
plot(t,tpos-imu_P)
hold on
 plot(t,tpos-epos,'--','linewidth',2)
plot(t,left_con*0.1,':m','linewidth',2.5)
plot(t,right_con*0.1,':c','linewidth',2.5)
grid on
title('Pos States - Truth')
grid on

subplot(spa,spb,4)
plot(t,tvel-imu_V)
grid on
title('INS velocity errors')


% here we plot the estimated accelerometer biases
figure(8); clf

subplot(spa,spb,1)
plot(t, force_)
grid on
title('force_ in the world frame (think so)')


subplot(spa,spb,2)
plot(t,biasa);
grid on
title('Accelerometer Biases');
hold on
plot(t, zvu_flag,'m:','linewidth',2);

subplot(spa,spb,3)
plot(t,dpos_bias_fb);
grid on
title('delta pos contribution to acc bias')
axis([0 t(end) -.01 0.01])

subplot(spa,spb,4)
plot(t,dvel_bias_fb);
grid on
title('delta vel contribution to acc bias')



% figure(7)


    figure(9)

    subplot(spa,spb,1)

    plot(tpos-epos), grid on, xlabel('Sim time [s]'), title('position errors')

    subplot(spa,spb,2)
    errv = tvel-evel;
    plot(t,errv), grid on, title(['velocity errors, std=', num2str(norm(std(errv((floor(0.1*end)):end)))) ' m/s'])
    

    subplot(spa,spb,3)
    plot(t,(tE-eE)*r2d), grid on, title('Euler angle errors [deg]')

    
%     plot(t,(trate-erate)*r2d), grid on,title('rate errors [deg/s]')
if (true)
    [bpx,bpy] = ginput();
    
    subplot(spa,spb,4)
    f9_d = detrend((tpos-epos),'linear',floor(bpx));

    plot(t(1:(length(f9_d))),f9_d)
    grid on
    title('detrended position state estimation errors ')

end


%% adjust all plots

%plot at so many sigma
sigma_plot = 3;

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
if (true)
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

