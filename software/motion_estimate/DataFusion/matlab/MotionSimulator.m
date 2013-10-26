function RESULTS = MotionSimulator(iterations, trajtype)

if (nargin<2)
    trajtype = 'atlas';
end

%% Setup lcm

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_INS_POSE_STATE', aggregator);


%% Prepare IMU data



RESULTS = setupUnitTest1Results(iterations);

param.gravity = 9.81; % this is in the forward left up coordinate frame
param.dt = 1E-3;




switch (trajtype) 
    case 'atlas'
        RESULTS.traj = gen_traj(iterations, param,0);
    case 'rotate'
        RESULTS.traj = gen_specifc_traj(iterations, param, 0);
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

%%

data{iterations} = [];

% for n = 1:iterations
%     data{n}.true.utime = traj.utime(n);
%     data{n}.true.inertial.ddp = GM_accels(n,:)';
%     data{n}.true.inertial.da = GM_rates(n,:)';
%     data{n}.true.environment.gravity = [0;0;9.81];% using forward-left-up/xyz body frame
    % add earth rate here
    % add magnetics here
% end
    

imuMsgBatch = [];
for n = 1:15
    imuMsgBatch = [imuMsgBatch, drc.atlas_raw_imu_t()];
end

% Setup the results data structure

%% Send and IMU messages

% Set initial pose states to zero
pose = init_pose();
storeCppINS.pose = init_pose();

data.true.utime = RESULTS.traj.utime;

data.true.pose.utime = RESULTS.traj.utime;
data.true.pose.P_l = RESULTS.traj.true.P_l;
data.true.pose.V_l = RESULTS.traj.true.V_l;
data.true.pose.f_l = RESULTS.traj.true.f_l;
% for n=1:iterations
% data.true.pose.R(n,:,:) = q2R(traj.true.q(n,:)');
% end
    
% Assume constant gravity for the test
true.inertial.gravity = [0;0;RESULTS.traj.parameters.gravity];% using forward-left-up/xyz body frame



disp('Starting the motion simulation loop')
for n = 1:iterations
    
    true.inertial.utime =  RESULTS.traj.utime(n);
    true.inertial.ddp   =  RESULTS.traj.true.a_b(n,:)';
    true.inertial.da    = -RESULTS.traj.true.w_b(n,:)';
    true.inertial.q     =  RESULTS.traj.true.q(n,:)';
    
    % Compute the truth trajectory
    if (n==1)
       % start with the correct initial conditions (first iteration -- init conditions are kept in pose)
       trueINS.pose = INS_Mechanisation(pose, true.inertial);
    else
        % normal operation
       trueINS.pose = INS_Mechanisation(trueINS.pose, true.inertial);
       
    end
    
    
    
    % add earth bound effects, including gravity
    measured.imu.utime =  RESULTS.traj.utime(n);
    measured.imu.gyr   =  RESULTS.traj.true.w_b(n,:)';
    measured.imu.acc   =  RESULTS.traj.true.a_b(n,:)';
    measured.imu.q     =  RESULTS.traj.true.q(n,:)';
    
    % Add sensor errors
%     [measured.imu, noiseStates ] = addIMUNoise(measured.imu, noiseStates)
    % measured.imu.gyr = measured.imu.gyr + ;
    
     % stimulus to a separate state-estimate process, send AtlasBatchIMU
     % data via LCM message
    [imuMsgBatch,sentIMUBatch] = sendDrcAtlasRawIMU(param.dt,n, measured,imuMsgBatch,lc);
    % here we listen back for an INS state message from the state estimate
    % processThis is the new way of working.
    
    % data{n}.INS.pose = receivepose(aggregator);
    if (sentIMUBatch)
        [cppINS, dummy] = receiveInertialStatePos(aggregator);
        cppINS.pose.E = q2e(cppINS.pose.q);
        storeCppINS = cppINS;
    end
    
    % ====== Here we evaluate the results of the data =====================
    % store the MATLAB computed INS pose states
    RESULTS.trueINSPose.P_l(n,:) = trueINS.pose.P_l';
    RESULTS.trueINSPose.V_l(n,:) = trueINS.pose.V_l';
    RESULTS.trueINSPose.f_l(n,:) = trueINS.pose.f_l';
    RESULTS.trueINSPose.a_l(n,:) = trueINS.pose.a_l';
    
    RESULTS.trueINSPose.q(n,:) = trueINS.pose.q;
    RESULTS.trueINSPose.E(n,:) = trueINS.pose.E';
    
    % store the cpp computed INS pose states
    RESULTS.cppINSPose.P_l(n,:) = storeCppINS.pose.P_l';
    RESULTS.cppINSPose.V_l(n,:) = storeCppINS.pose.V_l';
    RESULTS.cppINSPose.f_l(n,:) = storeCppINS.pose.f_l';
    RESULTS.cppINSPose.a_l(n,:) = storeCppINS.pose.a_l';
    
    RESULTS.cppINSPose.q(n,:) = storeCppINS.pose.q;
    RESULTS.cppINSPose.E(n,:) = storeCppINS.pose.E';
    
    
    % These are the residuals from the control test INS here in MATLABland
    RESULTS.trueINSPoseResiduals.P_l(n,:) = RESULTS.traj.true.P_l(n,:) - trueINS.pose.P_l';
    RESULTS.trueINSPoseResiduals.V_l(n,:) = RESULTS.traj.true.V_l(n,:) - trueINS.pose.V_l';
    RESULTS.trueINSPoseResiduals.f_l(n,:) = RESULTS.traj.true.P_l(n,:) - trueINS.pose.f_l';
    
    RESULTS.trueINSPoseResiduals.q(n,:) = quaternionResidual(RESULTS.traj.true.q(n,:)', trueINS.pose.q')';
    
    % These are the residuals from the state-estimate process
    RESULTS.cppINSPoseResiduals.P_l(n,:) = RESULTS.traj.true.P_l(n,:) - storeCppINS.pose.P_l';
    RESULTS.cppINSPoseResiduals.V_l(n,:) = RESULTS.traj.true.V_l(n,:) - storeCppINS.pose.V_l';
    RESULTS.cppINSPoseResiduals.f_l(n,:) = RESULTS.traj.true.f_l(n,:) - storeCppINS.pose.f_l';
    
    RESULTS.cppINSPoseResiduals.q(n,:) = quaternionResidual(RESULTS.traj.true.q(n,:), storeCppINS.pose.q')';
    
end

disp('Out of loop')


% We exit early because we are not ready to plot the result yet
% return






