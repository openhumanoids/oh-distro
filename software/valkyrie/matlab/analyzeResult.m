try
    i=0;
    rrtTimes = [];
    smoothingTimes = [];
    while true
        i = i + 1;
        load(sprintf('testResults/scene1_%03d.mat',i))
        rrtTimes(end+1) = simVars.rrtTime;
        smoothingTimes(end+1) = simVars.smoothingTime;
    end
catch
end