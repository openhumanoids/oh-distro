function batchRRT(n)
    scenes = {'scene1', 'scene2', 'scene3'};
    warning('off', 'MATLAB:MKDIR:DirectoryExists');
    for scene = 1:length(scenes)
        %Create a directory to save the results
        mkdir(scenes{scene});
        cd(scenes{scene});
        
        %Set up the simulation, run it n times and save the results
        options.scene = scenes{scene};
        options.visualize = false;
        for i = 1:n
            [xtraj, info, v, simVars] = exploringRRT(options);
            save(sprintf('%03d', i), 'simVars')
        end
        % back to the parent dir
        cd ..
    end
    warning('on', 'MATLAB:MKDIR:DirectoryExists');
end