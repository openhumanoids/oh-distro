data = csvread('/home/toby/Desktop/drc-logs/vrc_task_3_complete_historicfirstrun_11thJune_1053PM-joint-pos-diffs.csv');
%data = csvread('/home/toby/Desktop/drc-logs/single_committed_manip_map_message-joint-pos-diffs.csv');

names = {'back_lbz', 'back_mby', 'back_ubx','neck_ay','l_leg_uhz','l_leg_mhx','l_leg_lhy','l_leg_kny','l_leg_uay','l_leg_lax','r_leg_uhz','r_leg_mhx','r_leg_lhy','r_leg_kny','r_leg_uay','r_leg_lax','l_arm_usy','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_uwy','l_arm_mwx','r_arm_usy','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_uwy','r_arm_mwx'};

x = -2*3.15:0.01:2*3.15;
x_full = -2*3.142:0.001:2*3.142;

n_all = NaN(size(data, 2)+1, length(x_full));
n_all(1,:) = x_full;



for j = 1:size(data,2)
    subplot(7,4,j);
    [n xout] = hist(data(:,j), x);
    ni = interp1(xout,n,x_full);
    bar(x_full, ni);
    n_all(j+1,:) = ni + 1;
    
    xlim([-2*3.142 2*3.142])
    title(names{j}, 'Interpreter', 'none');
    xlabel('position delta');
    ylabel('frequency');
end

csvwrite('/home/toby/drc/software/network/network_sim/src/network_bridge/joint_pos_frequencies.csv', n_all);
