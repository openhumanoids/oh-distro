% data = csvread('/home/toby/Desktop/drc-logs/vrc_task_3_complete_historicfirstrun_11thJune_1053PM-joint-pos-diffs.csv');


x = -2*3.15:0.01:2*3.15;
x_full = -2*3.142:0.001:2*3.142;

n_all = NaN(2, length(x_full));
n_all(1,:) = x_full;

a=10000;
c=0.1;

n_all(2,:) = 1+ a*exp(-x_full.^2/(2*c^2));
plot(n_all(1,:), n_all(2,:))


csvwrite('/home/toby/drc/software/network/src/drc_network_shaper/2015_aggregate_joint_pos_frequencies.csv', n_all);
