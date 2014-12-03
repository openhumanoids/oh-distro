function drakeAtlasSimulationWithHands
%NOTEST

visualize = true; % should be a function arg

% load in Atlas initial state
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
%options.ignore_friction = true;
options.dt = 0.005;
options.visualize = visualize;
% boxes = [1.0, 0.0, 1.2, 1, 0.15;
%          1.2, 0.0, 0.8, 1, 0.30;];
% options.terrain = RigidBodyStepTerrain(boxes);
% TODO: get from LCM
options.hokuyo = false; % don't need sensors on control copy
options.foot_force_sensors = false;
options.hands = 'robotiq_weight_only';
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
% This is the one that has all of the neat important simul attributes:
options.hokuyo = true;
options.hokuyo_spin_rate = 4;
options.foot_force_sensors = false; % This works (you'll have to change
                                    % LCMBroadcastBlock to broadcast them)
                                    % but is slow right now.
options.hands = 'robotiq';
r_hands = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r_hands = r_hands.removeCollisionGroupsExcept({'heel','toe', 'palm', 'knuckle', 'default'});
r = compile(r);
r_hands = compile(r_hands);

% Add something to grab to r_hands
options_cyl.floating = true;
%won't work until floating joints supported in addRobotFromURDF
%r_hands = r_hands.addRobotFromURDF('manip_world_ex.urdf', [0; 0; 0]);
r_hands = r_hands.addRobotFromURDF('table.urdf', [1.225; 0.0; 0.5]);
r_hands = r_hands.addRobotFromURDF('cylinder.urdf', [0.775; -0.2; 1.07], [], options_cyl);
r_hands = compile(r_hands);

% set initial state to fixed point
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar(1) = 0;
xstar(2) = 0;
xstar(6) = 0;
xstar(28) = xstar(28) - 0.05;
%xstar(23) = xstar(23) - 0.1;

% and hand state to a feasible sol
% so lcp has an easier time
xstar_hands_init = r_hands.getInitialState();
xstar_hands = zeros(r_hands.getNumStates, 1);
xstar_hands(1:length(xstar_hands_init)) = xstar_hands_init;
xstar_hands(1:length(xstar)) = xstar;

xstar_hands = r_hands.resolveConstraints(xstar_hands);
r_hands = r_hands.setInitialState(xstar_hands);

xstar = xstar_hands(1:length(xstar));
x0 = xstar;
r = r.setInitialState(x0);

% LCM interpret in
ins(1).system = 2;
ins(1).input = 2;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
outs(3).system = 2;
outs(3).output = 3;
outs(4).system = 2;
outs(4).output = 4;
lcmAtlasInputBlock = LCMInputFromAtlasCommandBlock(r_hands, r,options);
sys = mimoFeedback(lcmAtlasInputBlock, r_hands, [], [], ins, outs);

clear ins
lcmRobotiqInputBlock = LCMInputFromRobotiqCommandBlock(r_hands, options);
sys = mimoFeedback(lcmRobotiqInputBlock, sys, [], [], [], outs);

% LCM broadcast out
lcmBroadcastBlock = LCMBroadcastBlock(r_hands, r);
sys = mimoCascade(sys, lcmBroadcastBlock);


% Visualize if desired
if visualize
  v = r_hands.constructVisualizer;
  v.display_dt = 0.1;
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  output_select(2).system=1;
  output_select(2).output=2;
  output_select(3).system=1;
  output_select(3).output=3;
  % might be wrong
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end

% TODO is to move to runLCM so that I don't  have to handle LCM
% parsing within a mimoOutput function. For now that's functioning
% and gives me some useful control over how that's working...
xstar_hands(3) = xstar_hands(3) + 0.01;
simulate(sys,[0.0,Inf], xstar_hands, options);
