function [xtraj, info, v, simVars, statVars] = exploringRRT(options, rng_seed)
% TIMEOUT 600
if nargin < 1 || isempty(options), options = struct(); end
if nargin > 1
    rng(rng_seed);
end
rndSeed = rng;
%save lastRndg.mat rndSeed

w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
if ~isfield(options,'scene'), options.scene = 'scene2'; end;
if ~isfield(options,'model'), options.model = 'v4'; end;
if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
switch options.model
    case 'v3'
        if options.convex_hull
            urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_convex_hull_robotiq_hands.urdf');
        else            
            urdf = fullfile(getDrakePath(),'..','models','atlas_v3','model_LR_RR.urdf');
        end
    case 'v4'
        urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_convex_hull_fingers.urdf');
    case 'v5'
        urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_convex_hull_fingers.urdf');
    case 'lwr'
        urdf = fullfile(getDrakePath(),'..','models','lwr_defs','robots','lwr_drake.urdf');
%     case 'val'
%         urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_mit_drake.urdf');
    otherwise
        urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
end
        
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = Scenes.getFP(options.model);
q_nom = S.xstar(1:nq);

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');

r = Scenes.generate(options, r, world);

r = r.compile();
warning(w);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = r.constructVisualizer();
if options.visualize
    v.draw(0, q_nom);
end

%Set IK options
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

%Set start pose constraints and compute starting configuration
startPoseConstraints = [{Scenes.addQuasiStaticConstraint(r)}, Scenes.fixedFeetConstraints(options, r)];
[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
if options.visualize
    v.draw(0, q_start);
end

%Set end pose constraints and compute end configuration
endPoseConstraints = [startPoseConstraints, Scenes.addGoalConstraint(options, r)];
if strcmp(options.scene, 'scene 3')
    endPoseConstraints = [endPoseConstraints, {Scenes.pelvisHeightConstraint(options,r, -.2)}];
end
[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, endPoseConstraints{:}, ikoptions);
if options.visualize
    v.draw(0, q_end);
end

%Create RRTs
min_distance = 0.01;
active_collision_options.body_idx = setdiff(1:r.getNumBodies(),[l_foot,r_foot]);
options.display_after_every = 100;
point_in_link_frame = [0; 0.4; 0];
TA = TaskSpaceMotionPlanningTree(r, 'l_hand', point_in_link_frame);
TA  = TA.setMinDistance(min_distance);
TA  = TA.setOrientationWeight(1);
TA.max_edge_length = 0.05;
TA.max_length_between_constraint_checks = TA.max_edge_length;
TA.angle_tol = 1*pi/180;
TA.position_tol = 1e-3;
TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), [l_foot, r_foot]);
TA.trees{TA.cspace_idx}.ikoptions = ikoptions;

kinsol = r.doKinematics(q_start);
xyz_quat_start = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,l_hand,point_in_link_frame,2);
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

if (info == 1) 
    if (options.n_smoothing_passes > 0)
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
    simVars.info = info;
    statVars.rrtTime = rrt_time;
    statVars.smoothingTime = smoothing_time;
    statVars.TAn = TA.n;
    statVars.TBn = TB.n;
    statVars.TCn = simVars.TConnected.n;
    statVars.TSn = T_smooth.n;
    statVars.info = info;
    q_path = extractPath(T_smooth, path_ids_A);
    TSlengths = q_path(1:3,2:end)-q_path(1:3, 1:end-1);
    statVars.TSlength = sum(diag(sqrt(TSlengths'*TSlengths)));
    q_pathC = extractPath(simVars.TConnected, path_ids_C);
    TClengths = q_pathC(1:3,2:end)-q_pathC(1:3, 1:end-1);
    statVars.TClength = sum(diag(sqrt(TClengths'*TClengths)));
    statVars.options = options;
    %distance

    if options.visualize
        v.playback(xtraj);
    end
else
    xtraj = [];
    v = [];
    simVars.info = info;
    statVars.info = info;
end

end


