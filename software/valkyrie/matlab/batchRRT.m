function batchRRT(scene, n, outfilename)

options.scene = scene;
options.visualize = false;
rrtTimes = zeros(1,n);
smoothingTimes = zeros(1,n);
for i = 1:n
    [xtraj, info, v, simVars] = exploringRRT(options);
    save(sprintf('%s_%03d',outfilename, i), 'simVars')
    rrtTimes(i) = simVars.rrtTime;
    smoothingTimes(i) = simVars.smoothingTime;
end
for i=1:n
    fprintf('Iteration %d:\n\tRRT time: %5.2f s\n\tSmoothing time: %5.2f s\n',...
        i, rrtTimes(i), smoothingTimes(i))
end
