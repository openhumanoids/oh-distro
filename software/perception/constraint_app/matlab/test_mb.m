function test_mb()
    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear functions;  % this clears all mex files from memory (and deletes all old objects)
    
    ptr = cam_initialize();