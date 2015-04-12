function drakeAtlasSimulation(atlas_version, visualize, add_hokuyo, right_hand, left_hand, world_name)
%NOTEST
if nargin < 1, atlas_version = 4; end
if nargin < 2, visualize = false; end
if nargin < 3, add_hokuyo = true; end
if nargin < 4, right_hand = 0; end
if nargin < 5, left_hand = 0; end
if nargin < 6, world_name = ''; end

right_hand = 4;
left_hand = 4;

% IF YOU WANT MASS EST LOOK HERE
% (when this is more fleshed out this will become
% an argument)
use_mass_est = false;

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct a pure atlas model for control
options.visualize = false; %don't use lcmgl lidar visualization... it's very slow
options.atlas_version = atlas_version;
options.floating = true;
options.dt = 0.00333;
options.hokuyo = false;
options.foot_force_sensors = false; % This works (you'll have to change
% LCMBroadcastBlock to broadcast them)
% but is slow right now.
options.obstacles = false; % Replaced by step terrain, though the option still works...
if (right_hand)
  options.hand_right = 'robotiq_weight_only';
end
if (left_hand)
  options.hand_left = 'robotiq_weight_only';
end
if (strcmp(world_name,'steps'))
  boxes = [1.0, 0.0, 1.2, 1, 0.15;
    1.2, 0.0, 0.8, 1, 0.30;];
  options.terrain = RigidBodyStepTerrain(boxes);
else
  options.terrain = RigidBodyFlatTerrain();
end
r_pure = DRCAtlas([],options);
% And construct a complete one
if (add_hokuyo)
  options.hokuyo = true;
  options.hokuyo_spin_rate = 4;
end
options.foot_force_sensors = false; % This works (you'll have to change
% LCMBroadcastBlock to broadcast them)
% but is slow right now.
sdfDir = fullfile(getDrakePath, 'examples', 'Atlas', 'sdf');
terrainSDF = fullfile(sdfDir,'drc_practice_task_2.world');

options.hand_right = getHandString(right_hand);
options.hand_left = getHandString(left_hand);

if (strcmp(world_name,'steps'))
  boxes = [1.0, 0.0, 1.2, 1, 0.15;
    1.2, 0.0, 0.8, 1, 0.30;];
  options.terrain = RigidBodyStepTerrain(boxes);
elseif (strcmp(world_name, 'terrain'))
  clear gazeboModelPath;
  setenv('GAZEBO_MODEL_PATH',sdfDir);
  height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(RigidBodyManipulator(terrainSDF),[],-3:.015:10,-2:.015:2,10);
  options.terrain = height_map;
  options.use_bullet = false;
elseif (strcmp(world_name, 'stairs'))
  stairsRBM = RigidBodyManipulator();
  stairsRBM = stairsRBM.setTerrain(RigidBodyFlatTerrain());
  stairsRBM = stairsRBM.addRobotFromURDF('stairs.urdf', [1.5 ; 0.0; 0.0], [0 ; 0 ; pi]);
  stairsRBM = stairsRBM.compile();
  height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(stairsRBM,[],-3:.015:3, -3:.015:3, 10);
  options.terrain = height_map;
  options.use_bullet = false;
else
  options.terrain = RigidBodyFlatTerrain();
end
options.replace_cylinders_with_capsules = false;
r_complete = DRCAtlas([],options);

% simplify collision to parts we care about
r_pure = r_pure.removeCollisionGroupsExcept({'heel','toe'});
r_complete = r_complete.removeCollisionGroupsExcept({'heel','toe', 'palm', 'knuckle', 'default'});
r_pure = compile(r_pure);

% we need to add a FT sensor frame
ft_link_ind = r_complete.getManipulator.findLinkId('r_hand');
ftframe = RigidBodyFrame(ft_link_ind,[0;0;0],[0;0;0],'force_torque_sensor');
r_complete = r_complete.addFrame(ftframe);
r_complete = r_complete.compile();


% set initial state to fixed point
S = load(r_pure.fixed_point_file);
xstar = S.xstar;
xstar(1) = 0;
xstar(2) = 0;
xstar(3) = xstar(3);
xstar(6) = 0;
x0 = zeros(r_pure.getNumStates, 1);
x0(1:length(xstar)) = xstar;
r_pure = r_pure.setInitialState(x0);

% and complete state to a feasible sol
% so lcp has an easier time
xstar_complete = zeros(r_complete.getNumStates(), 1);
xstar_complete(1:length(xstar)) = xstar;
xstar_complete = r_complete.resolveConstraints(xstar_complete);
r_complete = r_complete.setInitialState(xstar_complete);

T_wheel = [0.528863 6.31613e-10 0.848707 0.387817; 
        9.23685e-08 1 -5.83027e-08 0.462541; 
        -0.848707 1.09228e-07 0.528863 1.06883; 
        0 0 0 1];
radius = 0.17;


clear drivingPlanner;
clear options;
options.wheel_radius = radius;
options.R = rpy2rotmat([0;0;0]);
q0 = xstar_complete(1:36);
dp = drivingPlanner(r_complete,T_wheel,q0,options);
v = r_complete.constructVisualizer;
keyboard;


end

function handString = getHandString(hand_id)
  switch hand_id
    case 1
      handString = 'robotiq';
    case 2
      handString = 'robotiq_tendons';
    case 3
      handString = 'robotiq_simple';
    otherwise
      handString = 'none';
  end
end

function handDriver = getHandDriver(hand_id, r_complete, handedness, options)
  switch hand_id
    case 1
      handDriver = LCMInputFromRobotiqCommandBlock(r_complete, handedness, options);
    case 2
      handDriver = LCMInputFromRobotiqCommandBlockTendons(r_complete, handedness, options);
    case 3
      handDriver = LCMInputFromRobotiqCommandBlockSimplePD(r_complete, handedness, options);
    otherwise
      handDriver = [];
      disp('unexpected hand type, should be {1, 2, 3}')
  end
end