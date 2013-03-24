function test()
    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear functions;
    
    ptr = cam_initialize();
    
    fprintf('running...\n');
    
    try 
        while(true)
            success = cam_waitForObservations(ptr, 1000);
            if success
                fprintf('got new data\n');
                [success, expected, actual] = cam_getObservations(ptr)
            else
                fprintf('timed out waiting for data\n');
            end
        end
    catch me
        me
    end
    
    cam_destroy(ptr);
    fprintf('stopped.\n');