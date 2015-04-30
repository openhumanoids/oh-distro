function batchRRT(n, options)
    
    if nargin < 2, options.visualize = true; end
    
    path= fileparts(which('batchRRT'));
    cd(path)
    scenes = {'scene1', 'scene2', 'scene3'};
    warning('off', 'MATLAB:MKDIR:DirectoryExists');
    warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
    warning('off', 'MATLAB:Java:ConvertFromOpaque')
    for scene = 1:length(scenes)
        %Create a directory to save the results
        mkdir(scenes{scene});
        cd(scenes{scene});
        
        %Set up the simulation, run it n times and save the results
        options.scene = scenes{scene};
        for i = 1:n
            fprintf('Starting %s iteration %d ...\n', scenes{scene}, i)
            try
                [xtraj, info, v, simVars, statVars] = exploringRRT(options);
                save(sprintf('%03d', i), 'simVars', 'statVars')
                clear xtraj info v simVars statVars
            catch err
                fprintf('\tScene %s itereation %d failed with the following error:\n\t\t%s\n\tStack:\n',...
                     scenes{scene}, i, err.message)
                for st = 1:length(err.stack)
                    fprintf('\t\t%s in line %d\n', err.stack(st).name, err.stack(st).line)
                end
            end
            fprintf('Finished %s iteration %d ...\n', scenes{scene}, i)
        end
        % back to the parent dir
        cd ..
    end
    warning('on', 'MATLAB:MKDIR:DirectoryExists');
    warning('on', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
    warning('on', 'MATLAB:Java:ConvertFromOpaque')
end
