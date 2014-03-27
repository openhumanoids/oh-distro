clear all;

inds_mode = 0;


options = optimset('TolX',1e-4,'TolFun',1e-12,'Algorithm','interior-point','Display','iter-detailed'); % run interior-point algorithm

visualize_likelihood_space = 0;

% utimes = [1336827938975543;...
%           1336828091011880];
% logFile = '/home/abry/fixie_logs/fixie-gps-2012-05-12/fixie/lcmlog-2012-05-12.02';
% fileid = 'fixie-gps-2012-05-12.02';

utimes = [1230811579200651;...
          1230811775372596];
logFile = '/home/abry/fixie_logs/calm-outdoors-11-05-03/11-05-03-outdoors.01-powercutoff_translated';
fileid = '11-05-03-outdoors.01';


% window_sizes = 20:20:4000;
window_sizes = [50:50:200 300:100:1000 1200:200:2000];
% indow_sizes = linspace(20,4000,20);
num_window_sizes = length(window_sizes)
fileid = [fileid '-nw' num2str(num_window_sizes)]

opt_vecs = zeros(2,num_window_sizes);

n_q_gyro = 5;
n_q_accel = n_q_gyro + 1;

win_likelihoods = zeros(n_q_accel,n_q_gyro,num_window_sizes);

win_likelihoods_vel = zeros(n_q_accel,n_q_gyro,num_window_sizes);
win_likelihoods_chi = zeros(n_q_accel,n_q_gyro,num_window_sizes);
win_likelihoods_pos = zeros(n_q_accel,n_q_gyro,num_window_sizes);

q_vec_init_guess = [.5;.1];
q_vec_min = q_vec_init_guess/3;
q_vec_max = q_vec_init_guess*3;

temp_smoothed_file = [pods_get_data_path '/lcmlog-noise-opt-smoothed.tmp'];

init_char = 'g';
paramFile = sprintf('%s/fixie.cfg',pods_get_config_path);
system_smoothing_call = sprintf('%s/mav-state-estimator -i%s -S -L%s -l%s -P%s',...
    pods_get_bin_path, init_char, logFile, temp_smoothed_file, paramFile);

system(system_smoothing_call);
[neg_like,errors,error_covs] = noiseParamLikelihoodMex(q_vec_init_guess,window_sizes(end),inds_mode,utimes,temp_smoothed_file);

for kk = 1:num_window_sizes
    window_size = window_sizes(kk)
    tic    
    ins_noise_likelihood_anon_param = @(q_vec)noiseParamLikelihoodMex(q_vec,window_size,inds_mode,utimes);

    
%     if kk > 1
%         q_vec_init_guess = opt_vecs(:,kk-1);
%     end
    opt_q_vec = fmincon(ins_noise_likelihood_anon_param,q_vec_init_guess,[],[],[],[],.01*q_vec_min,100*q_vec_max,[],options)

    opt_vecs(:,kk) = opt_q_vec;
    
    toc

    q_gyros = linspace(q_vec_min(1),q_vec_max(1),n_q_gyro);
    q_accels = linspace(q_vec_min(2),q_vec_max(2),n_q_accel);

    [Q_GYROS,Q_ACCELS] = meshgrid(q_gyros,q_accels);

    likelihoods = zeros(n_q_accel,n_q_gyro);
% 
%     likelihoods_vel = zeros(n_q_accel,n_q_gyro);
%     likelihoods_chi = zeros(n_q_accel,n_q_gyro);
%     likelihoods_pos = zeros(n_q_accel,n_q_gyro);
% 


    if visualize_likelihood_space
    
        for ii=1:n_q_gyro
            for jj = 1:n_q_accel
                [likelihoods(jj,ii)] = ins_noise_likelihood_anon_param([q_gyros(ii);q_accels(jj)]);

            end
        end

        win_likelihoods(:,:,kk) = likelihoods;

%         win_likelihoods_vel(:,:,kk) = likelihoods_vel;
%         win_likelihoods_chi(:,:,kk) = likelihoods_chi;
%         win_likelihoods_pos(:,:,kk) = likelihoods_pos;


  
        figure(1);clf;
        subplot(2,2,1);
        contourf(q_gyros,q_accels,likelihoods);
        hold on
        colorbar
        xlabel('q gyro (deg/s)')
        ylabel('q accel (g)');
        grid on
        title(['log likelihood: opt = ' num2str(opt_q_vec(1)) ',' num2str(opt_q_vec(2)) ', N = ' num2str(window_size)]);

%         subplot(2,2,2);
%         contourf(q_gyros,q_accels,likelihoods_vel);
%         hold on
%         colorbar
%         xlabel('q gyro (deg/s)')
%         ylabel('q accel (g)');
%         grid on
%         title('v_b log likelihood')
% 
%         subplot(2,2,3);
%         contourf(q_gyros,q_accels,likelihoods_chi);
%         hold on
%         colorbar
%         xlabel('q gyro (deg/s)')
%         ylabel('q accel (g)');
%         grid on
%         title('\chi log likelihood')
% 
%         subplot(2,2,4);
%         contourf(q_gyros,q_accels,likelihoods_pos);
%         hold on
%         colorbar
%         xlabel('q gyro (deg/s)')
%         ylabel('q accel (g)');
%         grid on
%         title('\Delta log likelihood')
% 
% 
%         saveas(gcf,['q_log_like_' fileid '_' num2str(kk)],'png');
%         saveas(gcf,['q_log_like_' fileid '_' num2str(kk)],'fig');
% 
%         save([fileid 'q_log_like_vs_window_sizes.mat']);
%         
%         close all
    
    end
    toc
    
end

% save([fileid 'q_log_like_vs_window_sizes.mat']);

figure(6);clf;
plot(window_sizes/100,opt_vecs(1,:));
hold on
grid on
xlabel('look forward time (s)')
ylabel('optimal q gyro (deg/s)');
% saveas(gcf,['q_log_like_opt_gyro_' fileid],'png');
% saveas(gcf,['q_log_like_opt_gyro_' fileid],'fig');

figure(7);clf;
plot(window_sizes/100,opt_vecs(2,:));
hold on
grid on
xlabel('look forward time (s)')
ylabel('optimal q accel (g)');
% saveas(gcf,['q_log_like_opt_accel_' fileid],'png');
% saveas(gcf,['q_log_like_opt_accel_' fileid],'fig');
