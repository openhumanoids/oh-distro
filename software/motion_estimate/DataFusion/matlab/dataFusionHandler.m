function DFRESULTS = dataFusionHandler()
% Handles the data fusion computations as data is fed from
% state-estimation process. This scipts interacts through a LCM interface
% only.

% This is temporary
iterations = 12000/2;

% feedbackGain dictates how much of the parameter estimate we actually feed
% back into the INS solution (choose this parameter wisely, or it will bite you)
feedbackGain = 0.5;

dfSys.T = 0;

ENABLE_FEEDBACK = 1;
DataLogging = 0;

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

if (DataLogging == 1)
    DFRESULTS.STATEX = zeros(iterations,15);
    DFRESULTS.STATECOV = zeros(iterations,15);
    DFRESULTS.poses = [];
    DFRESULTS.REQMSGS = [];
    DFRESULTS.REQMSGS = initDFReqMsgData(iterations, DFRESULTS.REQMSGS);
    DFRESULTS.updatePackets = []; % temporary logging
end

index = 0;


% We assume this loop runs at 50 Hz or less 
while (true)
    
    index = index + 1;
    % wait for message
    [Measurement.INS, Measurement.LegOdo, DFReqMsg] = receiveInertialStatePos(aggregator);
    tic;
    % Now we can start computation
    % Ensure that we are not exceeding our allotted computation time
    if ((computationTime > 0.01) && DataLogging~=1)
        disp(['WARNING -- dataFusionHandler, long computation time ' num2str(computationTime*1000) ' ms']) 
    end

    if (dfSys.T ~= 0)
        dfSys.dt = 1E-6*(Measurement.INS.pose.utime - dfSys.T);% this should be taken from the utime stamps when ported to real data
    else
        dfSys.dt = 1E-3;
    end
    dfSys.T = Measurement.INS.pose.utime;
    
    %Measurement.positionResidual = Measurement.LegOdo.pose.P_l - Measurement.INS.pose.P_l;
    Measurement.velocityResidual = Measurement.LegOdo.pose.V_l - Measurement.INS.pose.V_l;
    
    %Measurement.quaternionManifoldResidual = R2q(q2R(Measurement.INS.pose.lQb)' * q2R(Measurement.LegOdo.pose.lQb));
    

    [Result, dfSys] = iterate([], dfSys, Measurement);

    
    % Store stuff for later plotting
    if (DataLogging == 1)
        %DFRESULTS.REQMSGS = storeDFReqMsgData(index, DFRESULTS.REQMSGS, DFReqMsg);
        DFRESULTS.REQMSGS.utime(index) = DFReqMsg.utime;
        DFRESULTS.REQMSGS.ba(index,1:3) = [DFReqMsg.accBiasEst.x, DFReqMsg.accBiasEst.y, DFReqMsg.accBiasEst.z];
        DFRESULTS.REQMSGS.bg(index,1:3) = [DFReqMsg.gyroBiasEst.x, DFReqMsg.gyroBiasEst.y, DFReqMsg.gyroBiasEst.z];
        DFRESULTS.REQMSGS.a_b(index,1:3) = [DFReqMsg.predicted_a_b.x, DFReqMsg.predicted_a_b.y, DFReqMsg.predicted_a_b.z];
        DFRESULTS.REQMSGS.w_b(index,1:3) = [DFReqMsg.predicted_w_b.x, DFReqMsg.predicted_w_b.y, DFReqMsg.predicted_w_b.z];
        DFRESULTS.REQMSGS.a_l(index,1:3) = [DFReqMsg.local_linear_acceleration.x, DFReqMsg.local_linear_acceleration.y, DFReqMsg.local_linear_acceleration.z];
        DFRESULTS.REQMSGS.f_l(index,1:3) = [DFReqMsg.local_linear_force.x, DFReqMsg.local_linear_force.y, DFReqMsg.local_linear_force.z];
        DFRESULTS.REQMSGS.P_l(index,1:3) = [DFReqMsg.pose.translation.x, DFReqMsg.pose.translation.y, DFReqMsg.pose.translation.z];
        DFRESULTS.REQMSGS.V_l(index,1:3) = [DFReqMsg.twist.linear_velocity.x, DFReqMsg.twist.linear_velocity.y, DFReqMsg.twist.linear_velocity.z];
        DFRESULTS.REQMSGS.lQb(index,1:4) = [DFReqMsg.pose.rotation.w, DFReqMsg.pose.rotation.x, DFReqMsg.pose.rotation.y, DFReqMsg.pose.rotation.z];
        DFRESULTS.poses = storePose(Measurement.INS.pose, DFRESULTS.poses, index);
        DFRESULTS.STATEX(index,:) = dfSys.posterior.x';
        DFRESULTS.STATECOV(index,:) = diag(dfSys.posterior.P);
        DFRESULTS.updatePackets = [DFRESULTS.updatePackets; feedbackGain*dfSys.posterior.x(4:6)'];
    end
    
    
    % Here we need to publish an INS update message -- this is caught by
    % state-estimate process and incorporated in the INS there
    if (ENABLE_FEEDBACK == 1)
        publishINSUpdatePacket(INSUpdateMsg, dfSys.posterior, feedbackGain, lc);
        dfSys.posterior.x = (1-feedbackGain) * dfSys.posterior.x;
    end
    
    computationTime = toc;
    if (Measurement.INS.pose.utime == (120 * 1E6) )
        break;
    end
end



%% Here we want to plot some dataFusion results.

if (DataLogging == 1)
    plotGrayINSPredicted(DFRESULTS.REQMSGS, 1);
    plotEKFResults(DFRESULTS, 2)
end

% Must standardize this plotting

% figure(10), clf
% subplot(321),plot(DFRESULTS.STATEX(:,4:6)),title('Gyro Bias Estimate'), grid on
% subplot(323),semilogx(DFRESULTS.STATEX(:,13:15)),title('Acc Bias Estimate'), grid on
% 
% 
% subplot(322),loglog(sqrt(DFRESULTS.STATECOV(:,4:6))),title('predicted gyro bias residual covariance'), grid on
% subplot(324),loglog(sqrt(DFRESULTS.STATECOV(:,13:15))),title('predicted acc bias residual covariance'), grid on


