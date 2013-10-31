function dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.


% Initialize local variables
computationTime = 0;

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_MATLAB_DATAFUSION_REQ', aggregator);

% initialize the Kalman Filter

posterior.x = zeros(15,1);
posterior.P = 999*eye(15);

% We assume that this loop will be run at no more than 50 Hz -- although we
% still need to test and validate this. The assumption is based on normal
% complimentary filtering practices from the Aerospace community.
while (true)
    % wait for message
    [Measurement.INS, Measurement.LegOdo] = receiveInertialStatePos(aggregator);
    
    % Now ew can start computation
    tic;
%     disp('Received data fusion request')
    % Ensure that we are not exceeding our allotted computation time
    if (computationTime > 0.045)
        disp(['WARNING -- dataFusionHandler is taking longer than 40ms, time taken was' num2str(computationTime)]) 
    end

    Sys.T = 0.02;% this should be taken from the utime stamps when ported to real data
    Sys.posterior = posterior;

    Measurement.positionResidual = Measurement.LegOdo.pose.P_l - Measurement.INS.pose.P_l;
    Measurement.quaternionLinearDiff = Measurement.LegOdo.pose.q - Measurement.INS.pose.q; % We do not intend to use the linear difference between the quaternions, just a sanity check
    
    disp(['Position residual ' num2str(Measurement.positionResidual')])
    disp(['Linear difference in quaternions ' num2str(Measurement.quaternionLinearDiff')])
    
    [Result, dfSys] = iterate([], Sys, Measurement);

    posterior = dfSys.posterior;

    computationTime = toc;
    if (Measurement.INS.pose.utime == 5000000)
        break;
    end
end



