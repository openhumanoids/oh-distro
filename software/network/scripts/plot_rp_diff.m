csv_data1 = csvread('/home/toby/Desktop/drc-logs/vrc_task_3_complete_historicfirstrun_11thJune_1053PM-rpy.csv');
csv_data2 = csvread('/home/toby/Desktop/drc-logs/footstep-lcmlog-2013-05-29-00-rpy.csv');

csv_data = [csv_data1; repmat(csv_data2, 12, 1)];

data = reshape(csv_data(:, 1:2), length(csv_data)*2,1);

x = -2*1.40:0.01:2*1.40;
x_full = -2*1.396:0.001:2*1.396;

n_all = NaN(size(data, 2)+1, length(x_full));
n_all(1,:) = x_full;


figure;
for j = 1:size(data,2)
    [n xout] = hist(data(:,j), x);
    ni = interp1(xout,n,x_full);
    bar(x_full, ni+1);
    n_all(j+1,:) = ni + 1;
    
    xlim([x(1) x(end)])
    xlabel('position delta');
    ylabel('frequency');
end

csvwrite('/home/toby/drc/software/network/network_sim/src/network_bridge/rollpitch_frequencies.csv', n_all);
