function DFRESULTS = dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.

% This is temporary
iterations = 20000/50;

% feedbackGain dictates how much of the parameter estimate we actually feed
% back into the INS solution (choose this parameter wisely, or it will bite you)
feedbackGain = 0.2;

ENABLE_FEEDBACK = 1;

% Initialize local variables
computationTime = 0;

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_MATLAB_DATAFUSION_REQ', aggregator);

INSUpdateMsg = initINSUpdateLCMMsg();

% initialize the Kalman Filter

% state vector
% x = [dPsi dbg dV dba dP]
dfSys.posterior.x = zeros(15,1);
dfSys.posterior.P = blkdiag(1*eye(2), [0.05], 0.1*eye(2), [0.1], 1*eye(3), 0.01*eye(3), 0*eye(3));

DFRESULTS.STATEX = zeros(iterations,15);
DFRESULTS.STATECOV = zeros(iterations,15);

index = 0;

% We assume this loop runs at 50 Hz or less 
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

    dfSys.T = 0.02;% this should be taken from the utime stamps when ported to real data
    %dfSys.posterior = posterior;

    Measurement.positionResidual = Measurement.LegOdo.pose.P_l - Measurement.INS.pose.P_l;
    Measurement.velocityResidual = Measurement.LegOdo.pose.V_l - Measurement.INS.pose.V_l;
    
    Measurement.quaternionLinearResidual = Measurement.LegOdo.pose.q - Measurement.INS.pose.q; % We do not intend to use the linear difference between the quaternions, just a sanity check
    Measurement.quaternionManifoldResidual = R2q(q2R(Measurement.INS.pose.q)' * q2R(Measurement.LegOdo.pose.q));
    
    disp(['dataFusionHandler -- Local frame Position residual ' num2str(Measurement.positionResidual')])
    disp(['dataFusionHandler -- Local frame Velocity residual ' num2str(Measurement.velocityResidual')])
    
    %     disp(['dataFusionHandler -- Manifold residual in quaternion norm: ' num2str(norm(Measurement.quaternionManifoldResidual)) ', q = ' num2str(Measurement.quaternionManifoldResidual)])
   
    [Result, dfSys] = iterate([], dfSys, Measurement);

    
    % Here we need to publish an INS update message -- this is caught by
    % state-estimate process and incorporated in the INS there
    if (ENABLE_FEEDBACK == 1)
        % From the DFusion_Vel_LFBRR test script. Different implementation here
        %[ dfSys.posterior.x, INSCompensator ] = LimitedStateTransfer( posterior.x, limitedFB, INSCompensator );
        publishINSUpdatePacket(INSUpdateMsg, dfSys.posterior, feedbackGain, lc);
        % move error information to the INS solution and remove from state
        % estimate posterior. This is a feedback step.
        % Assume no packet loss, with the safety net of some additional
        % process noise and move to C++ implementation.
        % Graceful tranfer of estimated error state to compensate for 
        % linearization errors; also prevents orbitting of the solution.
        dfSys.posterior.x = (1-feedbackGain) * dfSys.posterior.x;
    end
    
    % Store data in the correct location -- We can streamline this once the
    % system is running properly
    %posterior = dfSys.posterior;
    
    % Store stuff for later plotting
    DFRESULTS.STATEX(index,:) = dfSys.posterior.x';
    DFRESULTS.STATECOV(index,:) = diag(dfSys.posterior.P);
    
    computationTime = toc
    if (Measurement.INS.pose.utime == (50 * iterations * 1000) )
        break;
    end
end



%% Here we want to plot some dataFusion results.

% Must standardize this plotting

figure(10), clf
subplot(321),semilogx(DFRESULTS.STATEX(:,4:6)),title('Gyro Bias Estimate'), grid on
subplot(323),semilogx(DFRESULTS.STATEX(:,13:15)),title('Acc Bias Estimate'), grid on


subplot(322),loglog(sqrt(DFRESULTS.STATECOV(:,4:6))),title('predicted gyro bias residual covariance'), grid on
subplot(324),loglog(sqrt(DFRESULTS.STATECOV(:,13:15))),title('predicted acc bias residual covariance'), grid on


