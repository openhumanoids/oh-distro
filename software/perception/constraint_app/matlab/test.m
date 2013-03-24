function test()
    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear functions;
    
    ptr = cam_initialize();
    
    fprintf('running...\n');
    
    try 
        maxId = 0;
        
        for i = 1:10
            success = cam_waitForObservations(ptr, 5000);
            if ~success
                fprintf('timed out waiting for data\n');
                continue;
            end
            
            %TODO: determine if we have been reset with a new fit.  if so,
            %clear the state estimate covariance matrix
            
            %TODO: x = getStateEstimate(ptr)

            [success, expected, actual, id] = cam_getObservations(ptr)
            if ~success
                fprintf('error while attempting to get observations\n');
                continue;
            end
            
            N = max([id maxId])+1;
            zk = nan(3*N,1);
            expected_zk = nan(3*N,1);
            id_mask = reshape(repmat(id, 3, 1) * 3 + repmat([0;1;2], 1, size(id,2)) + 1, [], 1);
            zk(id_mask) = actual;
            expected_zk(id_mask) = expected;
            
            %TODO: perform UKF
            
            %TODO: setStateEstimate(ptr, x)
        end
    catch me
        me
    end
    
    cam_destroy(ptr);
    fprintf('stopped.\n');