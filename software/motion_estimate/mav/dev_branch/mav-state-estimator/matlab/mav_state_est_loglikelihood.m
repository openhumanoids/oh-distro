function negloglike = mav_state_est_loglikelihood(param_vec,param_list,flags)

param_override_flags = '-O"';

for ii=1:length(param_vec)
    param_override_flags = [param_override_flags param_list{ii} '=' num2str(param_vec(ii)) '|'];
end
param_override_flags(end)='"';

log_output_file = [pods_get_data_path '/temp_mav_state_est_neglog_like'];
system_call = sprintf('%s/mav-state-estimator %s %s -M%s',...
    pods_get_bin_path, param_override_flags, flags, log_output_file)
system(system_call);

addpath_eigen_utils();
eigmat = -loadEigenMatrices(log_output_file);
negloglike = eigmat(1,1);
rmpath_eigen_utils();
system(['rm ' log_output_file]);