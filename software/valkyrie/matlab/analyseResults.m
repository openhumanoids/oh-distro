function [rrtTimes, smoothingTimes] = analyseResults(nFiles, statOnly, verbose)
    
    if nargin < 1, nFiles = 100; end
    if nargin < 2, statOnly = true; end
    if nargin < 3, verbose = false; end
    
    warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
    warning('off', 'MATLAB:class:mustReturnObject')
    
    path= fileparts(which('batchRRT'));
    scenes = {'scene1', 'scene2', 'scene3'};
    cd([path, '/', 'scene1'])
%     f = dir([path '/' 'scene1' '/***.mat']);
%     nFiles = length(f);

    %Unpack statVars
    try
        load('001.mat', 'statVars')
    catch err
        disp('Cannot open 001.mat')
        rethrow(err);
    end
    names = fieldnames(statVars);
    for i=1:length(names)
        n = names{i};
        eval([n '= zeros(nFiles, length(scenes));'])
    end
    %{
    meanRRT = zeros(length(scenes), 1);
    medianRRT = zeros(length(scenes), 1);
    stdevRRT = zeros(length(scenes), 1);
    meanSmoothing = zeros(length(scenes), 1);
    medianSmoothing = zeros(length(scenes), 1);
    stdevSmoothing = zeros(length(scenes), 1);
    meanTot = zeros(length(scenes), 1);
    medianTot = zeros(length(scenes), 1);
    stdevTot = zeros(length(scenes), 1);
    meanTAn = zeros(length(scenes), 1);
    medianTAn = zeros(length(scenes), 1);
    stdevTAn = zeros(length(scenes), 1);
    meanTBn = zeros(length(scenes), 1);
    medianTBn = zeros(length(scenes), 1);
    stdevTBn = zeros(length(scenes), 1);
    meanTCn = zeros(length(scenes), 1);
    medianTCn = zeros(length(scenes), 1);
    stdevTCn = zeros(length(scenes), 1);
    meanTSn = zeros(length(scenes), 1);
    medianTSn = zeros(length(scenes), 1);
    stdevTSn = zeros(length(scenes), 1);
    meanTClength = zeros(length(scenes), 1);
    medianTClength = zeros(length(scenes), 1);
    stdevTClength = zeros(length(scenes), 1);
    meanTSlength = zeros(length(scenes), 1);
    medianTSlength = zeros(length(scenes), 1);
    stdevTSlength = zeros(length(scenes), 1);
    nFail = zeros(length(scenes), 1);
    %}
    
    for scene = 1:length(scenes)
        cd([path, '/', scenes{scene}])
        f = dir([path '/' scenes{scene} '/***.mat']);
        nFail(scene) = nFiles - length(f);

        %% Data Analysis

        for i = 1:nFiles
            if verbose, fprintf('Loading %s iteration %d...\n', scenes{scene}, i); end
            try
                if statOnly
                    load(sprintf('%03d.mat',i), 'statVars')
                else
                    load(sprintf('%03d.mat',i))
                end
            catch
                continue;
            end
            for j=1:length(names)
                n = names{j};
                eval([n '(i, scene)= statVars.(names{j});'])
            end
        end
    end
    
    cd(path)
    
    %% Plot
    if exist('aboxplot', 'file') == 2
        gcf;
        clf;
        X(1,:,:) = rrtTime;
        X(2,:,:) = smoothingTime;
        X(3,:,:) = rrtTime + smoothingTime;
        aboxplot(X, 'labels', scenes, 'OutlierMarker', '+');
        ylabel('Time [s]')
        legend('RRT Time', 'Smoothing Time', 'Total Time')
        print('-deps', 'Times.eps')
        
        clf;
        clear X
        X(1,:,:) = TCn;
        X(2,:,:) = TSn;
        aboxplot(X, 'labels', scenes);
        ylabel('N points')
        legend('Before Smoothing', 'After Smoothing')
        print('-deps', 'nPoints.eps')
        
        clf;
        clear X
        X(1,:,:) = TClength;
        X(2,:,:) = TSlength;
        aboxplot(X, 'labels', scenes);
        ylabel('length [m]')
        legend('Before Smoothing', 'After Smoothing')
        print('-deps', 'length.eps')
    else
        gcf
        boxplot(rrtTime)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('RRT Time')
        print('-dpng', 'rrtTime.png')
        
        clf
        boxplot(smoothingTime)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('smoothing Time')
        print('-dpng', 'smoothingTime.png')
        
        clf
        boxplot(rrtTime + smoothingTime)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('Total Time')
        print('-dpng', 'rrtTime.png')
        
        clf
        boxplot(TCn)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('N points before Sampling')
        print('-dpng', 'TCn.png')
        
        clf
        boxplot(TSn)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('N points after Sampling')
        print('-dpng', 'TSn.png')
        
        clf
        boxplot(TClength)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('length before Sampling')
        print('-dpng', 'TClength.png')
        
        clf
        boxplot(TSlength)    
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('length after Sampling')
        print('-dpng', 'TSlength.png')

        %{
        figure()
        bar([meanTCn meanTSn])
        hold on
        legend({'N points (before smoothing)', 'N points (after smoothing)'})
        errorbar(1:length(scenes), meanTCn, stdevTCn, '.k')
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel(')
        print('-dpng', 'TCn.png')

        errorbar(1:length(scenes), meanTSn, stdevTSn, '.k')
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel()
        print('-dpng', 'TSn.png')

        errorbar(1:length(scenes), meanTot, stdevTot, '.k')
        set(gca, 'XtickLabel', scenes)
        set(gca, 'Xtick', 1:length(scenes))
        ylabel('Total Time')
        print('-dpng', 'TotTime.png')
        %}
    end

    fprintf('Analysis completed\n')

    warning('on', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
    warning('on', 'MATLAB:class:mustReturnObject')
end