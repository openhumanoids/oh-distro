function DFRESULTS = dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.

% This is temporary
iterations = 10000/50;

% feedbackGain dictates how much of the parameter estimate we actually feed
% back into the INS solution (choose this parameter wisely, or it will bite you)
feedbackGain = 1;

dfSys.T = 0;

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
DFRESULTS.poses = [];
DFRESULTS.REQMSGS = [];
DFRESULTS.REQMSGS = initDFReqMsgData(iterations, DFRESULTS.REQMSGS);

DFRESULTS.updatePackets = []; % temporary logging

index = 0;

DFRESULTS.REQMSGS.utime(end+1) = 0

% We assume this loop runs at 50 Hz or less 
while (true)
    index = index + 1;
    % wait for message
    [Measurement.INS, Measurement.LegOdo, DFReqMsg] = receiveInertialStatePos(aggregator);
    DFRESULTS.REQMSGS = storeDFReqMsgData(index, DFRESULTS.REQMSGS, DFReqMsg);
    
    % Now we can start computation
    tic;
%     disp('Received data fusion request')
    % Ensure that we are not exceeding our allotted computation time
    if (computationTime > 0.045)
        disp(['WARNING -- dataFusionHandler is taking longer than 40ms, time taken was' num2str(computationTime)]) 
    end

    if (dfSys.T ~= 0)
        dfSys.dt = 1E-6*(Measurement.INS.pose.utime - dfSys.T);% this should be taken from the utime stamps when ported to real data
    else
        dfSys.dt = 1E-2;
    end
    dfSys.T = Measurement.INS.pose.utime;
    
    %Measurement.positionResidual = Measurement.LegOdo.pose.P_l - Measurement.INS.pose.P_l;
    Measurement.velocityResidual = Measurement.LegOdo.pose.V_l - Measurement.INS.pose.V_l;
    
    %Measurement.quaternionManifoldResidual = R2q(q2R(Measurement.INS.pose.lQb)' * q2R(Measurement.LegOdo.pose.lQb));
    

    [Result, dfSys] = iterate([], dfSys, Measurement);

    
    % Store stuff for later plotting
    DFRESULTS.poses = storePose(Measurement.INS.pose, DFRESULTS.poses, index);
    DFRESULTS.STATEX(index,:) = dfSys.posterior.x';
    DFRESULTS.STATECOV(index,:) = diag(dfSys.posterior.P);
    
    % Here we need to publish an INS update message -- this is caught by
    % state-estimate process and incorporated in the INS there
    if (ENABLE_FEEDBACK == 1)

        publishINSUpdatePacket(INSUpdateMsg, dfSys.posterior, feedbackGain, lc);
        DFRESULTS.updatePackets = [DFRESULTS.updatePackets; feedbackGain*dfSys.posterior.x(4:6)'];
        dfSys.posterior.x = (1-feedbackGain) * dfSys.posterior.x;
    end
    
    computationTime = toc;
    if (Measurement.INS.pose.utime == (100 * 1000000) )
        break;
    end
end



%% Here we want to plot some dataFusion results.


plotGrayINSPredicted(DFRESULTS.REQMSGS, 1);

% Must standardize this plotting

% figure(10), clf
% subplot(321),plot(DFRESULTS.STATEX(:,4:6)),title('Gyro Bias Estimate'), grid on
% subplot(323),semilogx(DFRESULTS.STATEX(:,13:15)),title('Acc Bias Estimate'), grid on
% 
% 
% subplot(322),loglog(sqrt(DFRESULTS.STATECOV(:,4:6))),title('predicted gyro bias residual covariance'), grid on
% subplot(324),loglog(sqrt(DFRESULTS.STATECOV(:,13:15))),title('predicted acc bias residual covariance'), grid on


