function DFRESULTS = dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.

iterations = 5000/50;

% Initialize local variables
computationTime = 0;

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_MATLAB_DATAFUSION_REQ', aggregator);

% initialize the Kalman Filter

% States are misalignment and gyro bias
posterior.x = zeros(15,1);
posterior.P = 999*eye(15);

DFRESULTS.STATEX = zeros(iterations,15);
DFRESULTS.STATECOV = zeros(iterations,15);

index = 0;

% We assume that this loop will be run at no more than 50 Hz -- although we
% still need to test and validate this. The assumption is based on normal
% complimentary filtering practices from the Aerospace community.
while (true)
    index = index + 1;
    % wait for message
    [Measurement.INS, Measurement.LegOdo] = receiveInertialStatePos(aggregator);
    
    % Now we can start computation
    tic;
%     disp('Received data fusion request')
    % Ensure that we are not exceeding our allotted computation time
    if (computationTime > 0.045)
        disp(['WARNING -- dataFusionHandler is taking longer than 40ms, time taken was' num2str(computationTime)]) 
    end

    Sys.T = 0.02;% this should be taken from the utime stamps when ported to real data
    Sys.posterior = posterior;

    Measurement.positionResidual = Measurement.LegOdo.pose.P_l - Measurement.INS.pose.P_l;
    
    Measurement.quaternionLinearResidual = Measurement.LegOdo.pose.q - Measurement.INS.pose.q; % We do not intend to use the linear difference between the quaternions, just a sanity check
    Measurement.quaternionManifoldResidual = R2q(q2R(Measurement.INS.pose.q)' * q2R(Measurement.LegOdo.pose.q));
    
    disp(['dataFusionHandler -- Position residual ' num2str(Measurement.positionResidual')])
%     disp(['Linear difference in quaternion ' num2str(Measurement.quaternionLinearResidual')])
    disp(['dataFusionHandler -- Manifold residual in quaternion norm: ' num2str(norm(Measurement.quaternionManifoldResidual)) ', q = ' num2str(Measurement.quaternionManifoldResidual)])
   
    [Result, dfSys] = iterate([], Sys, Measurement);

    % Here we need to publish an INS update message
    
    posterior = dfSys.posterior;

    
    % Store stuff for later plotting
    DFRESULTS.STATEX(index,:) = posterior.x';
    DFRESULTS.STATECOV(index,:) = diag(posterior.P);
    
    computationTime = toc;
    if (Measurement.INS.pose.utime == (50 * iterations * 1000) )
        break;
    end
end



%% Here we want to plot some dataFusion results.

% We standardize our approach 

figure(10), clf
subplot(321),plot(DFRESULTS.STATEX(:,4:6)),title('Gyro Bias Estimate'), grid on
subplot(323),plot(DFRESULTS.STATEX(:,13:15)),title('Acc Bias Estimate'), grid on


subplot(322),loglog(sqrt(DFRESULTS.STATECOV(:,4:6))),title('predicted gyro bias residual covariance'), grid on
subplot(324),loglog(sqrt(DFRESULTS.STATECOV(:,13:15))),title('predicted acc bias residual covariance'), grid on


