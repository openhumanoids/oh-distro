
clc
clear

iterations = 1000;

% Run the full test
RESULTS = MotionSimulator(iterations,'rotate');


%% Plot Results

motionSimulatorPlotTraj3D(1,RESULTS.traj.true,RESULTS.trueINSPose,RESULTS.cppINSPose);
motionSimulatorPlotTrajComponents(2,RESULTS.traj.true,RESULTS.trueINSPose,RESULTS.cppINSPose);


return