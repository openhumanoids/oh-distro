%data = csvread('/home/toby/Desktop/drc-logs/vrc_task_3_complete_historicfirstrun_11thJune_1053PM-joint-pos-diffs.csv');
%data = csvread('/home/toby/Desktop/drc-logs/single_committed_manip_map_message-joint-pos-diffs.csv');

x = -2*3.15:0.01:2*3.15;
x_full = -2*3.142:0.001:2*3.142;

n_joints=30

n_all = NaN(n_joints+1, length(x_full));
n_all(1,:) = x_full;


a=1000;
c=0.25;
outside=10;

for j = 1:n_joints
    subplot(6,5,j);
    n_all(j+1,:) = round(outside+ a*exp(-x_full.^2/(2*c^2)));
    plot(n_all(1,:), n_all(j+1,:))
    
    xlim([-2*3.142 2*3.142])
    title(num2str(j), 'Interpreter', 'none');
    xlabel('position delta');
    ylabel('frequency');
end

csvwrite('/home/toby/drc/software/network/src/drc_network_shaper/2015_joint_pos_plan_frequencies.csv', n_all);
