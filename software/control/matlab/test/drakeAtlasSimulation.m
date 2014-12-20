function drakeAtlasSimulation(atlas_version)
%NOTEST
if nargin < 1, atlas_version = 4; end

visualize = false; % should be a function arg

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_friction = true;
options.dt = 0.00333;
options.visualize = visualize;
options.hokuyo = true;
options.hokuyo_spin_rate = 3;
options.foot_force_sensors = false; % This works (you'll have to change
                                    % LCMBroadcastBlock to broadcast them)
                                    % but is slow right now.
options.obstacles = false; % Replaced by step terrain
boxes = [1.0, 0.0, 1.2, 1, 0.15;
         1.2, 0.0, 0.8, 1, 0.30;];
options.terrain = RigidBodyStepTerrain(boxes);
options.atlas_version = atlas_version;
% TODO: get from LCM
%r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
r = Atlas([],options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
xstar = load(r.fixed_point_file);
xstar(1) = 0;
xstar(2) = 0;
xstar(3) = xstar(3) + 0.08;
xstar(6) = 0;
x0 = zeros(r.getNumStates, 1);
x0(1:length(xstar)) = xstar;
r = r.setInitialState(x0);

% LCM interpret in
outs(1).system = 2;
outs(1).output = 1;
if (options.hokuyo)
  outs(2).system = 2;
  outs(2).output = 2;
end
lcmInputBlock = LCMInputFromAtlasCommandBlock(r,[],options);
sys = mimoFeedback(lcmInputBlock, r, [], [], [], outs);

% LCM broadcast out
broadcast_opts = options;
broadcast_opts.publish_truth = 0;
broadcast_opts.publish_imu = 1;
lcmBroadcastBlock = LCMBroadcastBlock(r, r, broadcast_opts);
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
