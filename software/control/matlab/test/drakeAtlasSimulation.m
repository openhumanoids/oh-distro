function drakeAtlasSimulation(atlas_version, visualize, add_hokuyo, add_hands, world_name)
%NOTEST
if nargin < 1, atlas_version = 4; end
if nargin < 2, visualize = false; end
if nargin < 3, add_hokuyo = true; end
if nargin < 4, add_hands = false; end
if nargin < 5, world_name = ''; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

done = 0
while(~done)
  % construct a pure atlas model for control
  options.visualize = visualize;
  options.atlas_version = atlas_version;
  options.floating = true;
  options.dt = 0.00333;
  options.hokuyo = false;
  options.foot_force_sensors = false; % This works (you'll have to change
  % LCMBroadcastBlock to broadcast them)
  % but is slow right now.
  options.obstacles = false; % Replaced by step terrain, though the option still works...
  if (add_hands)
    options.hands = 'robotiq_weight_only';
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
  if (add_hands)
    options.hands = 'robotiq';
  end
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
  else
    options.terrain = RigidBodyFlatTerrain();
  end
  options.replace_cylinders_with_capsules = false;
  r_complete = DRCAtlas([],options);
  
  % simplify collision to parts we care about
  r_pure = r_pure.removeCollisionGroupsExcept({'heel','toe'});
  r_complete = r_complete.removeCollisionGroupsExcept({'heel','toe', 'palm', 'knuckle', 'default'});
  r_pure = compile(r_pure);
  
  % Add world if relevant
  if (strcmp(world_name, 'valve_wall'))
    % Add valve DRC environment
    r_complete = r_complete.addRobotFromURDF('single_valve_wall.urdf', [1.0; 0.0; 0.0], [0;0;pi]);
    r_complete = compile(r_complete);
  elseif (strcmp(world_name, 'drill_frame'))
    r_complete = r_complete.addRobotFromURDF([getDrakePath(), '/examples/Atlas/urdf/drill_frame.urdf'], [1.0; 0.0; 0], [0;0;pi]);
    r_complete = compile(r_complete);
  elseif (strcmp(world_name, 'door'))
    r_complete = r_complete.addRobotFromURDF([getDrakePath(), '/examples/Atlas/urdf/door.urdf'], [1.0; 0.0; 0], [0;0;pi]);
    r_complete = compile(r_complete);
  elseif (strcmp(world_name, 'manip_ex'))
    options_cyl.floating = true;
    r_complete = r_complete.addRobotFromURDF('table.urdf', [1.225; 0.0; 0.5]);
    r_complete = r_complete.addRobotFromURDF('drill_box.urdf', [0.775; -0.2; 1.2], [], options_cyl);
  elseif(strcmp(world_name, 'terrain'))
    r_complete = r_complete.addRobotFromSDF(terrainSDF);
  end
  r_complete = compile(r_complete);
  
  % set initial state to fixed point
  S = load(r_pure.fixed_point_file);
  xstar = S.xstar;
  xstar(1) = 0;
  xstar(2) = 0;
  xstar(3) = xstar(3) + 0.08;
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
  
  % Pass through outputs from robot
  for i=1:r_complete.getOutputFrame.getNumFrames
    outs(i).system = 2;
    outs(i).output = i;
  end
  % LCM intrepret Atlas commands
  lcmInputBlock = LCMInputFromAtlasCommandBlock(r_complete,r_pure,options);
  sys = mimoFeedback(lcmInputBlock, r_complete, [], [], [], outs);
  % LCM interpret in for hand
  if (add_hands)
    lcmRobotiqInputBlock = LCMInputFromRobotiqCommandBlock(r_complete, options);
    sys = mimoFeedback(lcmRobotiqInputBlock, sys, [], [], [], outs);
  end
  
  % LCM broadcast out
  broadcast_opts = options;
  broadcast_opts.publish_truth = 0;
  broadcast_opts.publish_imu = 1;
  lcmBroadcastBlock = LCMBroadcastBlock(r_complete,r_pure, broadcast_opts);
  sys = mimoCascade(sys, lcmBroadcastBlock);
  
  % Visualize if desired
  if visualize
    v = r_complete.constructVisualizer;
    v.display_dt = 0.01;
    S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
    output_select(1).system=1;
    output_select(1).output=1;
    sys = mimoCascade(sys,v,[],[],output_select);
    warning(S);
  end
  try
    simulate(sys,[0.0,Inf], xstar_complete, options);
  catch err
    disp('caught error in simulate(): restarting sim');
  end
end
