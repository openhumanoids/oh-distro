
clc
clear

iterations = 5000;

% Run the full test
RESULTS = MotionSimulator(iterations,'gyro_bias');


%% Plot Results

motionSimulatorPlotTraj3D(1,RESULTS.traj.true,RESULTS.trueINSPose,RESULTS.cppINSPose);
motionSimulatorPlotTrajComponents(2,RESULTS.traj.true,RESULTS.trueINSPose,RESULTS.cppINSPose);


return


