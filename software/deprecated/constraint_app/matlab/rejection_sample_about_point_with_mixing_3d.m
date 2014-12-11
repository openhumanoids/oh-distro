function [y, iterations] = rejection_sample_about_point_with_mixing_3d(x, J_at_x, pinvJ_at_x, chol_sysNoiseCov, chol_minimalSysNoiseCov, pointsDesired)

    % x - a 1xN point on the manifold, may or may not obey inequality constraints
    % pinvJ_at_x - inverse Jacobian NxM matrix at x
    % noiseCov - noise covariance matrix MxM in the embedding space (i.e., the observation space)
    % y - pointsDesiredxN points which lie on the manifold and obey the inequality constraints
    
    %N = size(x,2);
    N = size(J_at_x,2);
    M = size(chol_sysNoiseCov,1);
    
    null_projector_J_at_x = eye(N) - pinvJ_at_x * J_at_x;
    
    %vk = mvnrnd(zeros(1,N), minimalSysNoiseCov, pointsDesired);  % N x pointsDesired matrix of noise in state space (used only to the extent there is a null space in J_at_x)
    %wk = mvnrnd(zeros(1,M), sysNoiseCov, pointsDesired);  % M x pointsDesired matrix of noise in observation space
    vk = my_mvnrnd(chol_minimalSysNoiseCov, pointsDesired);
    wk = my_mvnrnd(chol_sysNoiseCov, pointsDesired);
    
    % TODO: this is adding a [twist g t l] vector together.  for twists,
    % this seems to be okay (judging by how KDL adds them).  might not
    % always be okay for every "incremental state"
    dx = pinvJ_at_x * wk' + null_projector_J_at_x * vk'; % this multiplies every row in wk by pinvJ to produce an N x pointsDesired matrix of projected noise in the tangent plane
    
    %y = repmat(x,pointsDesired,1) + dx';
    y = state_add(repmat(x,pointsDesired,1), dx');
    
%     chol_sysNoiseCov = cholcov(sysNoiseCov);
%     chol_minimalSysNoiseCov = cholcov(minimalSysNoiseCov);
    
    iterations = 0;
    while (1)
        [~, violation_id] = violations(y);
        badMask = violation_id ~= 0;
        numBadPoints = sum(badMask);
        if iterations == 100
            [~,vid] = violations(x);
            if vid ~= 0 
                x = enforceLimits(x);
            end
        end
        if numBadPoints > 0
            vk = my_mvnrnd(chol_minimalSysNoiseCov, numBadPoints);
            wk = my_mvnrnd(chol_sysNoiseCov, numBadPoints);
            dx = pinvJ_at_x * wk' + null_projector_J_at_x * vk';
            y(badMask,:) = state_add(repmat(x,sum(badMask),1), dx');
            iterations = iterations+1;
        else
            break;
        end        
    end
    
function R = my_mvnrnd(chol_Sigma, count)    
    R = randn(count, size(chol_Sigma,1)) * chol_Sigma;
    