function [xtraj, info, v, simVars, statVars] = exploringRRT(options, rng_seed)
% TIMEOUT 600
if nargin < 1 || isempty(options), options = struct(); end
if nargin < 2
    rndSeed = rng;
else
    rng(rng_seed);
end

w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
if ~isfield(options,'scene'), options.scene = 'debris'; end;
if ~isfield(options,'model'), options.model = 'v3'; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
switch options.model
    case 'v3'
        urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_convex_hull_robotiq_hands.urdf');
    case 'v4'
        urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_convex_hull_fingers.urdf');
    case 'v5'
        urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_convex_hull_fingers.urdf');
    case 'val'
        urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_mit_drake.urdf');
    otherwise
        urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
end
        
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1); %is it needed??

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

r = Scenes.generate(options.scene, r, world);

r = r.compile();
warning(w);


if strcmp(options.scene, 'debris')
        %Final position for the debrtis scene
        point_in_link_frame = [0; 0.24449999999999988; 0.011200000000000071];
        ref_frame = [0.10040853387866658, 0.30507204666777654, 0.94702121025152886, 0.19671872655867628; -0.070421541493923046, 0.95162340926777023, -0.29908810311880585, 1.0145817508809061; -0.9924509725008871, -0.036659695518642732, 0.11703475512224609, 0.9; 0.0, 0.0, 0.0, 1.0];
        lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
        upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];

        l_hand_final_pos_constraint = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
        %ref_frame = inv(ref_frame);
        %position_constraint_7 = WorldPositionConstraint(r, l_hand, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [1.0, 1.0]);

        l_hand_final_quat_constraint = WorldQuatConstraint(r, l_hand, [0.73638758447380859; 0.089093166809596377; 0.6584413641826542; -0.1274782451791375], 10*pi/180, [1.0, 1.0]);

else
        %Pos respect to the wrist frame
        point_in_link_frame = [0; 0.4; 0];
        
        %Reference frame for the final constraint
        ref_frame = [eye(3) Scenes.getTargetObjPos(options.scene)'; 0 0 0 1];
        
        lower_bounds = [0.0; 0.0; 0.0];
        upper_bounds = [0.0; 0.0; 0.0];
        
        %Final constraint
        l_hand_final_pos_constraint = WorldPositionInFrameConstraint(r, l_hand, point_in_link_frame, ref_frame,...
                                                               lower_bounds, upper_bounds, [1.0, 1.0]);
        l_hand_final_quat_constraint = WorldQuatConstraint(r, l_hand, axis2quat([0 0 1 -pi/2]'), 10*pi/180, [1.0, 1.0]);
        
        
        l_hand_initial_position_constraint = WorldPositionConstraint(r, l_hand, [0;0;0], xyz_quat_start(1:3), xyz_quat_start(1:3));

        l_hand_initial_quat_constraint = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0*pi/180);
end


min_distance = 0.01;
active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[l_foot,r_foot]);

options.display_after_every = 100;

TA = TaskSpaceMotionPlanningTree(r, 'l_hand', point_in_link_frame);
TA  = TA.setMinDistance(min_distance);
TA  = TA.setOrientationWeight(1);
TA.max_edge_length = 0.05;
TA.max_length_between_constraint_checks = TA.max_edge_length;
TA.angle_tol = 1*pi/180;
TA.position_tol = 1e-3;
TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), [l_foot, r_foot]);

ik_seed_pose = q_nom;
ik_nominal_pose = q_nom;
cost = Point(r.getPositionFrame(),10);
for i = r.getNumBodies():-1:1
  if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
    cost(r.getBody(r.getBody(i).parent).position_num) = ...
      cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
  end
end
cost = cost/min(cost);
Q = diag(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setMajorIterationsLimit(100);
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);
TA.trees{TA.cspace_idx}.ikoptions = ikoptions;


startPoseConstraints = Scenes.setStartPoseConstraints(options.scene, r);
[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);

if options.visualize
  v = r.constructVisualizer();
else
  v = [];
end

baseConstraint = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
baseConstraint = baseConstraint.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

%base_constraints = {quasiStaticConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint, rightFootQuatConstraint, backConstraint, baseConstraint,rightArmConstraint, leftLegConstraint};


active_constraints = [startPoseConstraints, {l_hand_final_pos_constraint,l_hand_final_quat_constraint}];
[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

if options.visualize
  v.draw(0,q_end);
end

kinsol = r.doKinematics(q_start);
xyz_quat_start = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
if options.visualize
  v.draw(0,q_start);
end


position_constraint_7 = WorldPositionConstraint(r, l_hand, point_in_link_frame, xyz_quat_start(1:3), xyz_quat_start(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, l_hand, xyz_quat_start(4:7), 0, [1.0, 1.0]);

active_constraints = [startPoseConstraints,{position_constraint_7,quat_constraint_8}];

[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);

x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];
xyz_box_edge_length = 2;
xyz_min = min(xyz_quat_start(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
xyz_max = max(xyz_quat_start(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;

TA = TA.setTranslationSamplingBounds(xyz_min, xyz_max);
TA = TA.addKinematicConstraint(startPoseConstraints{:});
TA = TA.setNominalConfiguration(q_nom);

TA = TA.compile();
TB = TA;
assert(TA.checkConstraints(x_start))
assert(TB.checkConstraints(x_goal))

% n_ee_poses_tried = 1;
%sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:},collision_constraint);
% sample_prog = InverseKinematics(r,ik_nominal_pose,base_constraints{:});
% sample_prog = sample_prog.setQ(0.1*ikoptions.Q);
% sample_prog = sample_prog.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_IterationsLimit);
% sample_prog.setSolverOptions('snopt','MajorFeasibilityTolerance',ikoptions.SNOPT_MajorFeasibilityTolerance);
% sample_prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-3);
TA = TA.setLCMGL('TA',[1,0,0]);
%TA.TB = TA.TB.setLCMGL('TA.TB',[1,0,0]);
TB = TB.setLCMGL('TB',[0,0,1]);
rrt_timer = tic;
%display('About to plan ...')
switch options.planning_mode
  case 'rrt'
    [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, TA, options);
  case 'rrt_connect'
    %display('Running RRTConnect')
    [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
end
rrt_time = toc(rrt_timer);
fprintf('  Timing:\n');
fprintf('    RRT:       %5.2f s\n', rrt_time);

T_smooth = TA;
simVars.TConnected = T_smooth;
path_ids_C = path_ids_A;
% interp_weight determines how much consideration is given to joint
% space distance during smoothing:
%  * 0 - end-effector distance only
%  * 1 - joint-space distance only
T_smooth.interp_weight = 0.5;
q_idx = TA.idx{TA.cspace_idx};

if (info == 1) && (options.n_smoothing_passes > 0)
  smoothing_timer = tic;
  T_smooth = T_smooth.setLCMGL('T_smooth', TA.line_color);
  [T_smooth, id_last] = T_smooth.recursiveConnectSmoothing(path_ids_A, options.n_smoothing_passes, options.visualize);
  path_ids_A = T_smooth.getPathToVertex(id_last);
  smoothing_time = toc(smoothing_timer);
  fprintf('    Smoothing: %5.2f s\n', smoothing_time);
  if options.visualize
    drawTree(TA);
    drawTree(TB);
    drawPath(T_smooth, path_ids_A);
  end
end

q_path = extractPath(T_smooth, path_ids_A);
path_length = size(q_path,2);

% Scale timing to obey joint velocity limits
% Create initial spline
q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(q_idx,:)));
t = linspace(0, 1, 10*path_length);
q_path = eval(q_traj, t);

% Determine max joint velocity at midpoint of  each segment
t_mid = mean([t(2:end); t(1:end-1)],1);
v_mid = max(abs(q_traj.fnder().eval(t_mid)), [], 1);

% Adjust durations to keep velocity below max
t_scaled = [0, cumsum(diff(t).*v_mid/mean(options.joint_v_max))];
tf = t_scaled(end);

% Warp time to give gradual acceleration/deceleration
t_scaled = tf*(-real(acos(2*t_scaled/tf-1)+pi)/2);
[t_scaled, idx_unique] = unique(t_scaled,'stable');

rState = r.getStateFrame();
xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
xtraj = xtraj.setOutputFrame(r.getStateFrame());

%Output structures
simVars.TA = TA;
simVars.TB = TB;
simVars.T_smooth = T_smooth;
simVars.path_ids_A = path_ids_A;
simVars.path_ids_C = path_ids_C;
simVars.rndSeed = rndSeed;
statVars.rrtTime = rrt_time;
statVars.smoothingTime = smoothing_time;
statVars.TAn = TA.n;
statVars.TBn = TB.n;
statVars.TCn = simVars.TConnected.n;
statVars.TSn = T_smooth.n;
q_path = extractPath(T_smooth, path_ids_A);
TSlengths = q_path(1:3,2:end)-q_path(1:3, 1:end-1);
statVars.TSlength = sum(diag(sqrt(TSlengths'*TSlengths)));
q_pathC = extractPath(simVars.TConnected, path_ids_C);
TClengths = q_pathC(1:3,2:end)-q_pathC(1:3, 1:end-1);
statVars.TClength = sum(diag(sqrt(TClengths'*TClengths)));
%distance

if options.visualize
    v.playback(xtraj);
end

end


