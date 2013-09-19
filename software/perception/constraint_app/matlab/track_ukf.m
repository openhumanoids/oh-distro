function track_ukf()
    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear java mex functions;  % this clears all mex files from memory (and deletes all old objects)
    
    setup_lcm();
    
    ptr = cam_initialize();
    lcm_ptr = lcm.lcm.LCM('udpm://239.255.76.67:7667?ttl=0');
    lcmgl = bot_lcmgl.LCMGL(lcm_ptr, 'track_ukf.m');
    
    fprintf('running...\n');
        
    %R = eye(8) * 1^2;  % system noise
    R = diag([0.01 0.1 0.1 0.1 0.1 0.1 1.0 1.0].^2);
    Q = [ 1 1 1 ] * 0.001^2;  % observation covariance; code assumes this is a diagonal matrix for conveience
    
    while ( true )
        success = cam_waitForObservations(ptr, 5000);
        if ~success
            fprintf('timed out waiting for data\n');
            continue;
        end
        
        %determine if we have been reset with a new fit.  if so,
        %  clear the state estimate covariance matrix
        if cam_getResetAndClear(ptr)
            fprintf('resetting filter\n');
            P = R;
            x = cam_getCurrentStateEstimate(ptr);  % get new estimate (from fit)
            fprintf('guess: \n');
            x
        else
            %TODO: perform update step; based on elapsed time
            x = cam_getCurrentStateEstimate(ptr);
            P = P + R;
        end
        prevX = x;
        
        [success, zk, id] = cam_getObservations(ptr);
        if ~success
            fprintf('error while attempting to get observations\n');
            continue;
        end                
        
        %perform UKF
        thisQ = diag(repmat(Q, 1, numel(zk)/3));
        [x, P, sigmaX, sigmaZ] = unscented_update(@(x)cam_getExpectedObservations(ptr, x, id), [], ...
            x', P, zk', thisQ);
        
        % update the code with a new estimate
        cam_setCurrentStateEstimate(ptr, x);
          
        % draw some debug output
        x'
        expected_z = cam_getExpectedObservations(ptr, x, id);
        expected_z = reshape(expected_z,3,[])';
        for i = 1:size(expected_z, 1)
            T = eye(4);
            T(1:3, 4) = expected_z(i,:)';
            lcmgl_DrawAxis(lcmgl, T, 0.2, 'b', 'b', 'b');
        end
        lcmgl.switchBuffers();
    end