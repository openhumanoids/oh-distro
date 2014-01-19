clear all;

options = optimset('TolX',1e-4,'TolFun',1e-12,'Algorithm','interior-point','Display','iter-detailed'); % run interior-point algorithm


logFile = '/home/abry/fixie_logs/fixie-gps-2012-05-12/fixie/lcmlog-2012-05-12.02';
fileid = 'fixie-gps-2012-05-12.02';
init_char = 'g';
paramFile = sprintf('%s/fixie.cfg',pods_get_config_path);

% logFile = '/home/abry/fixie_logs/calm-outdoors-11-05-03/11-05-03-outdoors.01-powercutoff_translated';
% fileid = '11-05-03-outdoors.01';
% init_char = 'g';
% paramFile = sprintf('%s/fixie.cfg',pods_get_config_path);


N=20;

flags = sprintf(' -i%s -L%s -P%s -R', init_char, logFile, paramFile);

param_list = {'state_estimator.gps.utime_offset'};
param_min_max = [.12*1e6, 0.2*1e6];

param_vals = linspace(param_min_max(1), param_min_max(2),N);
param_vals = round(param_vals);

negloglikes = zeros(1,N);

for ii=1:N
    negloglikes(ii) = mav_state_est_loglikelihood(param_vals(ii),param_list,flags);
end


figure(1);clf;
plot(param_vals,negloglikes);