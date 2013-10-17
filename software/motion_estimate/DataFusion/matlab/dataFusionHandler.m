function dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.


% Initialize local variables
computationTime = 0;

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('INS_ESTIMATE', aggregator);

% initialize the Kalman Filter

posterior.x = zeros(15,1);
posterior.P = 999*eye(15);

% We assume that this loop will be run at no more than 50 Hz -- although we
% still need to test and validate this. The assumption is based on normal
% complimentary filtering practices from the Aerospace community.
while (true)
    % wait for message
    [Measurement.INS.Pose, Measurement.LegOdo.Pose] = receiveInertialStatePos(aggregator);
    
    % Now ew can start computation
    tic;
    % Ensure that we are not exceeding our allotted computation time
    if (computationTime > 0.045)
        disp(['WARNING -- dataFusionHandler is taking longer than 40ms, time taken was' num2str(computationTime)]) 
    end

    Sys.T = 0.001;% this should be taken from the utime stamps when ported to real data
    Sys.posterior = posterior;

    Measurement.positionResidual = Measurement.LegOdo.Pose.P_l - Measurement.INS.Pose.P_l;
    
    [Result, data{n}.df] = iterate([], Sys, Measurement);

    posterior = data{n}.df.posterior;

    computationTime = toc;
end



