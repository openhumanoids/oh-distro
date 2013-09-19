function track_pf()

    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear java mex functions;  % this clears all mex files from memory (and deletes all old objects)
    
    %% setup lcm
    setup_lcm();
    
    ptr = cam_initialize();
    lcm_ptr = lcm.lcm.LCM('udpm://239.255.76.67:7667?ttl=0');
    lcmgl = bot_lcmgl.LCMGL(lcm_ptr, 'track_pf.m');
    
    msg = bot_param.update_t;
    msg.params = 'READY';
    lcm_ptr.publish('TRACK_PF_STATUS', msg)
    
    fprintf('running...\n');
    
%     %% parameteres specific to valve-wall task (2 wheels)
%     task.sysNoiseCov = diag( repmat([1,1,1], 1, 6) * 0.01^2 );       % noise for the state, expressed in observation space (over-parameterization)
%     task.minimalSysNoiseCov = diag( [1 1 1 1 1 1, 1 1] * 0.01^2 );   % noise for the state, expressed in state space    
%     task.obsNoiseCov = diag( repmat([1,1,1], 1, 6) * 0.005^2 );      % observation noise

    %% parameteres specific to valve-wall task (3 wheels)
    task.sysNoiseCov = diag( repmat([1,1,1], 1, 9) * 0.01^2 );       % noise for the state, expressed in observation space (over-parameterization)
    task.minimalSysNoiseCov = diag( [1 1 1 1 1 1, 1 1 1] * 0.01^2 );   % noise for the state, expressed in state space    
    task.obsNoiseCov = diag( repmat([1,1,1], 1, 9) * 0.005^2 );      % observation noise
    
    [~,s,v] = svd(inv(task.sysNoiseCov));
    task.sqrt_inv_sysNoiseCov = sqrt(s)*v';
    [~,s,v] = svd(inv(task.minimalSysNoiseCov));
    task.sqrt_inv_minimalSysNoiseCov = sqrt(s)*v';
    [~,s,v] = svd(inv(task.obsNoiseCov));
    task.sqrt_inv_obsNoiseCov = sqrt(s)*v';    
    
    task.sqrt_sysNoiseCov = cholcov(task.sysNoiseCov);
    task.sqrt_minimalSysNoiseCov = cholcov(task.minimalSysNoiseCov);
    
    %% setup partilce filter parameters
    pf.numOfParticlesForOISApproximation = 100;                                 % number of samples used to approximate the optimial importance function
    pf.Ns                                = 20;                                  % number of particles
    pf.task                              = task;
    pf.weightedMean                      = @(xk, wk)state_mean(xk', wk)';
    pf.gen_x0                            = @intial_particles;
    pf.importance_sampling               = @importance_sampling_wOIS_wMissing;
    
    k = 0;
    test = true;
    count = 0;
    tic;    
    while ( true )
        success = cam_waitForObservations(ptr, 5000);
        if ~success
            fprintf('timed out waiting for data\n');
            if test
                if k > 10
                    exit(0);
                else
                    exit(-1);  % if we have not processed 10 packets, and we're testing, then fail
                end
            else
                continue;
            end
        end
        
        %determine if we have been reset with a new fit.  if so,
        %  clear the state estimate covariance matrix
        if cam_getResetAndClear(ptr)
            fprintf('resetting filter\n');
            x = cam_getCurrentStateEstimate(ptr);  % get new estimate (from fit)
            % reset the particle filter
            pf.k = 0;
            pf.x0 = x;
            % print some debug info
            fprintf('guess: \n');
            x
        else
            x = cam_getCurrentStateEstimate(ptr);
        end
        prevX = x;
        
        [success, zk, observationIds] = cam_getObservations(ptr);
        pf.task.jacobian = @(x)cam_getJacobian(ptr, x, observationIds, 1);
        pf.task.get_obs = @(x)cam_getExpectedObservations(ptr, x', observationIds)';
        if ~success
            fprintf('error while attempting to get observations\n');
            continue;
        end                
        
        % perform particle filter update
        [x, pf] = particle_filter5(zk', pf, 'systematic_resampling');
        
        x(end-1:end) = wrapToPi(x(end-1:end));
        
        % update the code with a new estimate
        cam_setCurrentStateEstimate(ptr, x);
          
        % draw some debug output
        x'
        expected_z = cam_getExpectedObservations(ptr, x, observationIds);
        expected_z = reshape(expected_z,3,[])';
        for i = 1:size(expected_z, 1)
            T = eye(4);
            T(1:3, 4) = expected_z(i,:)';
            lcmgl_DrawAxis(lcmgl, T, 0.2, 'b', 'b', 'b');
        end
        lcmgl.switchBuffers();
        
        if mod(count,20)==0
            tic;
            count = 1;
        else
            count = count + 1;
            t = toc;
            fprintf('fps = %f\n', count / t);
        end
        k = k + 1;
    end
    
function x1 = intial_particles(xkm1, pointsDesired, task)   
    
    J_xkm1 = task.jacobian(xkm1);
    pinvJ_xkm1 = pinv_gunter(J_xkm1, 0.1);    
    x1 = rejection_sample_about_point_with_mixing_3d(xkm1, J_xkm1, pinvJ_xkm1, task.sqrt_sysNoiseCov, task.sqrt_minimalSysNoiseCov, pointsDesired);
    x1 = x1';
    
function [new_particles, weight_scales] = importance_sampling_wOIS_wMissing(xkm1, zk, pf)

    xkm1 = xkm1';
    zk = zk';
    
    % get expected observations at particles
    xkm1_dofState = pf.task.get_obs(xkm1);
    
    % difference between actual and expected observations
    zk_minus_xkm1 = obs_diff(repmat(zk, size(xkm1,1), 1), xkm1_dofState);
    
    % preallocate storage
    mk = zeros(size(zk_minus_xkm1,1), size(xkm1,2));
    xk = zeros(size(xkm1));
    weight_scales = zeros(size(xkm1,1),1);

    chol_inv_obsNoiseCov = pf.task.sqrt_inv_obsNoiseCov;
    
    for i = 1:size(xkm1,1)
        
        this_zk_minus_xkm1 = zk_minus_xkm1(i,:)';
        mk(i,:) = xkm1(i,:);
                
        p_zk_given_xkm1_history = zeros(3,1);
        for j = 1:size(p_zk_given_xkm1_history,1)
            J_xkm1 = pf.task.jacobian(mk(i,:));
            pinv_J_xkm1 = pinv_gunter(chol_inv_obsNoiseCov * J_xkm1, 0.1);
        
            this_zk_minus_xkm1(isnan(this_zk_minus_xkm1)) = 0;
            dx = pinv_J_xkm1 * chol_inv_obsNoiseCov * this_zk_minus_xkm1;
            mk(i,:) = state_add(mk(i,:), dx');  % note, mk may not obey the inequality constraints                    
            
            this_zk_minus_xkm1 = obs_diff(zk, pf.task.get_obs(mk(i,:)))';
        
            % create some sample points about mk; rejecting if not obeying the inequality constraints
            J_mk = pf.task.jacobian(mk(i,:));
            pinvJ_mk = pinv_gunter(J_mk, 0.1);
            [nk, ~] = rejection_sample_about_point_with_mixing_3d(mk(i,:), J_mk, pinvJ_mk, pf.task.sqrt_sysNoiseCov, pf.task.sqrt_minimalSysNoiseCov, pf.numOfParticlesForOISApproximation);

            % create some sample points about xkm1; rejecting if not obeying the inequality constraints
            pinvJ_xkm1 = pinv_gunter(J_xkm1, 0.1);
            [yk, ~] = rejection_sample_about_point_with_mixing_3d(xkm1(i,:), J_xkm1, pinvJ_xkm1, pf.task.sqrt_sysNoiseCov, pf.task.sqrt_minimalSysNoiseCov, pf.numOfParticlesForOISApproximation);

            % take all the above samples points, these form the xj.  nb: all of the xj are on the manifold and obey the inequality constraints
            xj = [nk; yk];
            xj_dofStates = pf.task.get_obs(xj);
            N_xj = size(xj,1);

            % evaluate wj = p(zk|xj)*p(xj|xkm1) for each xj.  this forms a new distribution approximating p(zk|xkm1) with particles xj and weights wj
            p_zkGivenxk = p_zk_given_xk(repmat(zk, N_xj, 1), xj_dofStates, chol_inv_obsNoiseCov);
            p_xkGivenxkm1 = p_xk_given_xkm1(xj_dofStates, repmat(xkm1_dofState(i,:), N_xj, 1), pf.task.sqrt_inv_sysNoiseCov) + 1e-6;
            p_zk_given_xj_times_p_xj_given_xkm1 = p_zkGivenxk .* p_xkGivenxkm1;
            p_zk_given_xkm1 = sum(p_zk_given_xj_times_p_xj_given_xkm1);
            p_xj_given_xkm1_and_zk = p_zk_given_xj_times_p_xj_given_xkm1 / p_zk_given_xkm1;

            p_zk_given_xkm1_history(j) = p_zk_given_xkm1;
            
            if p_zk_given_xkm1 > 1e-5
                break;
            end
        end

        % draw a sample, xk, from xj according to the weights wj (this differs from Grisetti who calculates a Gaussian and draws from the Gaussian)
        % the weight of the new particle is sum(wj)
        if p_zk_given_xkm1 <= 0
            % if every particle has a zero probability, just randomly select a particle.  the hope is some other particle will have better
            % luck.  if resampling occurs, this particle will be elimiated.
            xki = randsample(N_xj, 1);
        else
            xki = randsample(N_xj, 1, true, p_xj_given_xkm1_and_zk);
        end
        
        xk(i,:) = xj(xki,:);
        weight_scales(i) = p_zk_given_xkm1;  %nb: does not depend on xk chosen from xj
        
%         figure(101);
%         semilogy([p_zkGivenxk p_xkGivenxkm1 p_zk_given_xj_times_p_xj_given_xkm1])
%         legend('p(z_k|x_k)', 'p(x_k|x_k_-_1)', 'p(z_k|x_k)*p(x_k|x_k_-_1)', 'Location', 'Best')

        1;
    end

    new_particles = xk';
    
function p = p_zk_given_xk(zk, expected_zk_dq, sqrt_obsNoisePrecision)

    err = obs_diff(zk, expected_zk_dq);
    
    p = singular_mvnpdf(err, sqrt_obsNoisePrecision);
    
function p = p_xk_given_xkm1(xk_dofStates, xkm1_dofStates, sqrt_obsNoisePrecision)

    err = obs_diff(xk_dofStates, xkm1_dofStates);

    p = singular_mvnpdf(err, sqrt_obsNoisePrecision);    
   