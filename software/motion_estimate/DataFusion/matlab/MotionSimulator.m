function RESULTS = MotionSimulator(iterations, trajtype)

if (nargin<2)
    trajtype = 'atlas';
end

%% Setup lcm

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_INS_POSE_STATE', aggregator);


param.gravity = 9.8; % this is in the forward left up coordinate frame
param.dt = 1E-3;


%% Prepare IMU data

switch (trajtype) 
    case 'atlas'
        RESULTS.traj = gen_traj(iterations, param,0);
    case 'rotate'
        RESULTS.traj = gen_specifc_traj(iterations, param, trajtype);
    case 'gyro_bias'
        RESULTS.traj = gen_specifc_traj(iterations, param, trajtype);
    case 'acc_bias'
        RESULTS.traj = gen_specifc_traj(iterations, param, trajtype);
    case 'Microstrain_01'
        iterations = 6000;
        %RESULTS = setupUnitTest1Results(iterations);
        RESULTS.traj = load_specific_traj(iterations, 'UnitTests/testdata/dfd_loggedIMU_03.txt');
        param.dt = 1E-2;
end



%%

% This is what we have in the traj structure
%
% traj.iterations
% traj.utime
% traj.dt
% traj.parameters.gravity
% traj.true.P_l
% traj.true.V_l
% traj.true.f_l
% traj.true.f_b
% traj.true.a_l
% traj.true.a_b
% traj.true.w_l = w_l;
% traj.true.w_b = w_b;
% traj.true.E
% traj.true.q

%% Initialize the first empty IMU batch message

imuMsgBatch = [];
for n = 1:15
    imuMsgBatch = [imuMsgBatch, drc.atlas_raw_imu_t()];
end

% Setup the results data structure

%% Send and IMU messages

% prepare LCM messages

% trueMatlabPoseMsg = drc.nav_state_t();
% 
% trueMatlabPoseMsg.pose = drc.position_3d_t();
% trueMatlabPoseMsg.pose.translation = drc.vector_3d_t();
% trueMatlabPoseMsg.pose.rotation = drc.quaternion_t();
% trueMatlabPoseMsg.twist = drc.twist_t();
% trueMatlabPoseMsg.twist.linear_velocity = drc.vector_3d_t();
% trueMatlabPoseMsg.twist.angular_velocity = drc.vector_3d_t();
% trueMatlabPoseMsg.local_linear_acceleration = drc.vector_3d_t();
% 
% 
% % Set initial pose states to zero
% pose = init_pose();
% storeCppINS.pose = init_pose();
% 
% % This should be removed!!
% data.true.utime = RESULTS.traj.utime;
% data.true.pose.utime = RESULTS.traj.utime;
% data.true.pose.P_l = RESULTS.traj.true.P_l;
% data.true.pose.V_l = RESULTS.traj.true.V_l;
% data.true.pose.f_l = RESULTS.traj.true.f_l;
%     
% % Assume constant gravity for the test
% true.inertial.gravity = [0;0;param.gravity];% using forward-left-up/xyz body frame
% 
% receivedMsgs = 0;


% trueINS.pose = init_pose();
% trueINS.pose__k1 = init_pose();
% trueINS.pose__k2 = init_pose();

disp('Starting the motion simulation loop')
for n = 1:iterations
    n
    pause(0.001);
%     true.inertial.utime =  RESULTS.traj.utime(n);
%     true.inertial.a_b   =  RESULTS.traj.true.a_b(n,:)';
%     true.inertial.w_b   =  RESULTS.traj.true.w_b(n,:)';
%     true.inertial.lQb   =  RESULTS.traj.true.lQb(n,:)';
%     
%     % Compute the truth trajectory
%     trueINS.pose = INS_lQb([],trueINS.pose__k1, trueINS.pose__k2, true.inertial);
%     % COmpute MATLAB equivalent measurement based trajectory
%     trueINS.pose = INS_lQb([],pose__k1, pose__k2, true.inertial);
%     
%  
%     
%     % We also publish the truth data to whoever wants to listen -- this was
%     % added for the state-estimate process data fusion testing
%     truthpose.utime = RESULTS.traj.utime(n);
%     truthpose.P_l = RESULTS.traj.true.P_l(n,:)';
%     truthpose.lQb = RESULTS.traj.true.lQb(n,:)';
%     truthpose.V_l = RESULTS.traj.true.V_l(n,:)';
%     truthpose.w_l = RESULTS.traj.true.w_l(n,:)';
%     truthpose.f_l = RESULTS.traj.true.f_l(n,:)';
    
%     sendLCMMatlabTruthPose(truthpose, trueMatlabPoseMsg, lc);
    
    
    % add earth bound effects, including gravity
%     measured.imu.utime   =  RESULTS.traj.utime(n);
    measured.imu.utime   =  param.dt*n;
    measured.imu.gyr     =  RESULTS.traj.measured.w_b(n,:)';
    measured.imu.acc     =  RESULTS.traj.measured.a_b(n,:)';
%     measured.imu.lQb     =  RESULTS.traj.measured.lQb(n,:)';
    
    % Add sensor errors
%     [measured.imu, noiseStates ] = addIMUNoise(measured.imu, noiseStates)
    % measured.imu.gyr = measured.imu.gyr + ;
    
     % stimulus to a separate state-estimate process, send AtlasBatchIMU
     % data via LCM messageq
    [imuMsgBatch,sentIMUBatch] = sendDrcAtlasRawIMU(param.dt, n, measured,imuMsgBatch,lc);
    % here we listen back for an INS state message from the state estimate
    % processThis is the new way of working.
%     pause
%     if (sentIMUBatch)
%         receivedMsgs =  receivedMsgs + 1;
%         [cppINS, dummy] = receiveInertialStatePos(aggregator);
%         lqb = cppINS.pose.q;
%         bql = lqb;
%         bql(2:4) = -bql(2:4);
%         
%         cppINS.pose.E = q2e(bql);
%         storeCppINS = cppINS;
%     end
    
    % ====== Here we evaluate the results of the data =====================
    % store the MATLAB computed INS pose states
%     RESULTS.trueINSPose.P_l(n,:) = trueINS.pose.P_l';
%     RESULTS.trueINSPose.V_l(n,:) = trueINS.pose.V_l';
%     RESULTS.trueINSPose.f_l(n,:) = trueINS.pose.f_l';
%     RESULTS.trueINSPose.a_l(n,:) = trueINS.pose.a_l';
%     
%     RESULTS.trueINSPose.q(n,:) = trueINS.pose.q;
%     RESULTS.trueINSPose.E(n,:) = trueINS.pose.E';
%     
%     % store the cpp computed INS pose states
%     RESULTS.cppINSPose.P_l(n,:) = storeCppINS.pose.P_l';
%     RESULTS.cppINSPose.V_l(n,:) = storeCppINS.pose.V_l';
%     RESULTS.cppINSPose.f_l(n,:) = storeCppINS.pose.f_l';
%     RESULTS.cppINSPose.a_l(n,:) = storeCppINS.pose.a_l';
%     
%     RESULTS.cppINSPose.q(n,:) = storeCppINS.pose.q;
%     RESULTS.cppINSPose.E(n,:) = storeCppINS.pose.E';
%     
%     
%     % These are the residuals from the control test INS here in MATLABland
%     RESULTS.trueINSPoseResiduals.P_l(n,:) = RESULTS.traj.true.P_l(n,:) - trueINS.pose.P_l';
%     RESULTS.trueINSPoseResiduals.V_l(n,:) = RESULTS.traj.true.V_l(n,:) - trueINS.pose.V_l';
%     RESULTS.trueINSPoseResiduals.f_l(n,:) = RESULTS.traj.true.P_l(n,:) - trueINS.pose.f_l';
%     
%     RESULTS.trueINSPoseResiduals.q(n,:) = quaternionResidual(RESULTS.traj.true.q(n,:)', trueINS.pose.q')';
%     
%     % These are the residuals from the state-estimate process
%     RESULTS.cppINSPoseResiduals.P_l(n,:) = RESULTS.traj.true.P_l(n,:) - storeCppINS.pose.P_l';
%     RESULTS.cppINSPoseResiduals.V_l(n,:) = RESULTS.traj.true.V_l(n,:) - storeCppINS.pose.V_l';
%     RESULTS.cppINSPoseResiduals.f_l(n,:) = RESULTS.traj.true.f_l(n,:) - storeCppINS.pose.f_l';
%     
%     RESULTS.cppINSPoseResiduals.q(n,:) = quaternionResidual(RESULTS.traj.true.q(n,:), storeCppINS.pose.q')';
%     
%     
%     trueINS.pose__k2 = trueINS.pose__k1;
%     trueINS.pose__k1 = trueINS.pose;
    
end

disp('Out of loop')







