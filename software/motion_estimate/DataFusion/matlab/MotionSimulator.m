function RESULTS = MotionSimulator(iterations)

%% Setup lcm

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('SE_INS_POSE_STATE', aggregator);


%% Prepare IMU data

% iterations = 10000;

param.gravity = 9.81; % this is in the forward left up coordinate frame
param.dt = 1E-3;
    
traj = gen_traj(iterations, param,0);

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

%% Send and IMU messages

% Set initial pose states to zero
pose = init_pose();
storeCppINS.pose = init_pose();

data.true.utime = traj.utime;

data.true.pose.utime = traj.utime;
data.true.pose.P_l = traj.true.P_l;
data.true.pose.V_l = traj.true.V_l;
data.true.pose.f_l = traj.true.f_l;
% for n=1:iterations
% data.true.pose.R(n,:,:) = q2R(traj.true.q(n,:)');
% end
    
% Assume constant gravity for the test
true.inertial.gravity = [0;0;traj.parameters.gravity];% using forward-left-up/xyz body frame

% Setup the results data structure
RESUTLS = setupUnitTest1Results(iterations);

disp('Starting the motion simulation loop')
for n = 1:iterations
    
    true.inertial.utime =  traj.utime(n);
    true.inertial.ddp   =  traj.true.a_b(n,:)';
    true.inertial.da    = -traj.true.w_b(n,:)';
    true.inertial.q     =  traj.true.q(n,:)';
    
    % Compute the truth trajectory
    if (n==1)
       % start with the correct initial conditions (first iteration -- init conditions are kept in pose)
       trueINS.pose = INS_Mechanisation(pose, true.inertial);
    else
        % normal operation
       trueINS.pose = INS_Mechanisation(trueINS.pose, true.inertial);
       
    end
    
    % add earth bound effects, including gravity
    measured.imu.utime =  traj.utime(n);
    measured.imu.gyr   = -traj.true.w_b(n,:)';
    measured.imu.acc   =  traj.true.a_b(n,:)';
    measured.imu.q     =  traj.true.q(n,:)';
    
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
        storeCppINS = cppINS;
    end
    
    % ====== Here we evaluate the results of the data =====================
    
    % These are the residuals from the control test INS here in MATLABland
    RESULTS.trueINSPoseResiduals.P_l(n,:) = traj.true.P_l(n,:) - trueINS.pose.P_l';
    RESULTS.trueINSPoseResiduals.V_l(n,:) = traj.true.V_l(n,:) - trueINS.pose.V_l';
    RESULTS.trueINSPoseResiduals.f_l(n,:) = traj.true.P_l(n,:) - trueINS.pose.f_l';
    
    RESULTS.trueINSPoseResiduals.q(n,:) = quaternionResidual(traj.true.q(n,:)', trueINS.pose.q')';
    
    % These are the residuals from the state-estimate process
    RESULTS.cppINSPoseResiduals.P_l(n,:) = traj.true.P_l(n,:) - storeCppINS.pose.P_l';
    RESULTS.cppINSPoseResiduals.V_l(n,:) = traj.true.V_l(n,:) - storeCppINS.pose.V_l';
    RESULTS.cppINSPoseResiduals.f_l(n,:) = traj.true.f_l(n,:) - storeCppINS.pose.f_l';
    
    RESULTS.cppINSPoseResiduals.q(n,:) = quaternionResidual(traj.true.q(n,:), storeCppINS.pose.q')';
    
end

disp('Out of loop')


% We exit early because we are not ready to plot the result yet
% return






