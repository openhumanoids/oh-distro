function drakeAtlasSimulation
%NOTEST

visualize = false; % should be a function arg

% load in Atlas initial state
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.005;
options.visualize = visualize;
options.hokuyo = true;
options.foot_force_sensors = false; % This works (you'll have to change
                                    % LCMBroadcastBlock to broadcast them)
                                    % but is slow right now.
options.obstacles = false; % Replaced by step terrain
boxes = [1.0, 0.0, 1.2, 1, 0.15;
         1.2, 0.0, 0.8, 1, 0.30;];
options.terrain = RigidBodyStepTerrain(boxes);
% TODO: get from LCM
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 0;
xstar(2) = 0;
xstar(3) = xstar(3) + 0.12;
xstar(6) = 0;
x0 = zeros(r.getNumStates, 1);
x0(1:length(xstar)) = xstar;
r = r.setInitialState(x0);

% LCM interpret in
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
% outs(3).system = 2;
% outs(3).output = 3;
% outs(4).system = 2;
% outs(4).output = 4;
lcmInputBlock = LCMInputFromAtlasCommandBlock(r,options);
sys = mimoFeedback(lcmInputBlock, r, [], [], [], outs);

% LCM broadcast out
lcmBroadcastBlock = LCMBroadcastBlock(r);
sys = mimoCascade(sys, lcmBroadcastBlock);


% Visualize if desired
if visualize
  v = r.constructVisualizer;
  v.display_dt = 0.01;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end

% TODO is to move to runLCM so that I don't  have to handle LCM
% parsing within a mimoOutput function. For now that's functioning
% and gives me some useful control over how that's working...
simulate(sys,[0.0,Inf], [], options);
