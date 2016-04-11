function [xtraj,info,v] = testTaskSpaceRRTKuka( scene, options, rng_seed)
%NOTEST
% currently fails to smooth

% TIMEOUT 600
if nargin < 1; scene = 1; end
if nargin < 2 || isempty(options), options = struct(); end
if nargin < 3, rng; else rng(rng_seed); end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
fk_opts.rotation_type = 2;

options.hand_link_name = 'lwr_arm_7_link';

urdf = fullfile(getDrakePath(),'..','models','lwr_defs','robots','lwr_drake.urdf');
S = load([getDrakePath(), '/../control/matlab/data/kuka_lwr_fp.mat']);

% move the robot up as its in intersection with the world:
S.xstar(3) = 0.15

r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

world = r.findLinkId('world');
hand = r.findLinkId(options.hand_link_name);

% Box world:
%obstacle = RigidBodyBox([.5 .5 .5], [.45 0 0.7], [0 0 0]);
%obstacle = RigidBodyBox([.25 .25 .25], [.45 0 0.7], [0 0 0]);
%r = addGeometryToBody(r, world, obstacle);

% Capsule world:
collision_object = RigidBodyCapsule(0.2,0.5,[.45 0 0.7],[0,0,0]);
collision_object.c = [0.5;0.4;0.3];
r = addGeometryToBody(r, world, collision_object);

scene
% reach the hand from/to:
if (scene == 1)
%    start_xyz_quat = [0.0;0.0;0.8; rpy2quat([90*pi/180,0*pi/180,90*pi/180] ) ];
    start_xyz_quat = [0.175;0.4;0.95; rpy2quat([270*pi/180,-90*pi/180,-90*pi/180] ) ];
%    goal_xyz_quat = [0.0;0.0;0.9; rpy2quat([270*pi/180,0*pi/180,-90*pi/180] ) ];
    goal_xyz_quat = [0.525;-0.4;0.55; rpy2quat([270*pi/180,-90*pi/180,-90*pi/180] ) ];
%    goal_xyz_quat =  start_xyz_quat;%[0.1;0.0;0.0; rpy2quat([90*pi/180,0*pi/180,270*pi/180] ) ];
elseif (scene == 2)
    start_xyz_quat = [0.05;0.4;0.8; rpy2quat([-90*pi/180,60*pi/180,0*pi/180] ) ];
    goal_xyz_quat = [0.4;0.3;1.1; rpy2quat([-90*pi/180,0*pi/180,0*pi/180] ) ];
end

%%% Configuration (scene independent) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint_inds_base = [joints.base_x; joints.base_y; joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
%hand_to_palm_offset = [0.0; 0.0; 0.05]; 
hand_to_palm_offset = [0.0; 0.0; 0.0]; 
%inactive_collision_bodies = [l_foot,r_foot, LeftHipAdductor, RightHipAdductor,  LeftHipRotator, RightHipRotator, LowerNeckExtensor, LeftForearm, RightForearm, Head];


%%%%%% All the stuff above is set up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r = r.compile();
warning(w);

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joints_lower_limit = q_nom(joint_inds_base) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_nom(joint_inds_base) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds_base, joints_lower_limit, joints_upper_limit);

lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
goal_frame = xyzquat2frame(goal_xyz_quat);
position_constraint_7 = WorldPositionInFrameConstraint(r, hand, hand_to_palm_offset, goal_frame, lower_bounds, upper_bounds, [1.0, 1.0]);

hand_initial_position_constraint = WorldPositionConstraint(r, hand, [0;0;0], start_xyz_quat(1:3), start_xyz_quat(1:3));


temp = frame2xyzquat( goal_frame); quat_8 = temp(4:7);
quat_constraint_8 = WorldQuatConstraint(r, hand, quat_8, 0*pi/180, [1.0, 1.0]); % was 10*pi/180 , mfallon

hand_initial_quat_constraint = WorldQuatConstraint(r, hand, start_xyz_quat(4:7), 0*pi/180);

min_distance = 0.01;
% this seems to disable these joints from being used for collision checking:
%active_collision_options.body_idx = setdiff(1:r.getNumBodies(), inactive_collision_bodies);
active_collision_options.body_idx = setdiff(1:r.getNumBodies(), []);

options.display_after_every = 100;

%TA = TaskSpaceMotionPlanningTree(r, 'hand', hand_to_palm_offset);
TA = TaskSpaceMotionPlanningTree(r, options.hand_link_name, hand_to_palm_offset);

TA  = TA.setMinDistance(min_distance);
TA  = TA.setOrientationWeight(1);
TA.max_edge_length = 0.05;
TA.max_length_between_constraint_checks = TA.max_edge_length;
TA.angle_tol = 1*pi/180;
TA.position_tol = 1e-3;
% this seems to disable these joints from being used for collision checking:
%TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), inactive_collision_bodies);
TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), []);


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


active_constraints_start = {posture_constraint_6,  hand_initial_position_constraint , hand_initial_quat_constraint};
[q_start, info, infeasible_constraint_start] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints_start{:}, ikoptions);

if options.visualize
  v = r.constructVisualizer(); % robot arms spread, pelvise at zero
else
  v = [];
end

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y;joints.base_z; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

base_constraints = { posture_constraint_6};


active_constraints_end = [base_constraints, {position_constraint_7,quat_constraint_8}];
[q_end, info, infeasible_constraint_end] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints_end{:}, ikoptions);


if options.visualize
  v.draw(0,q_end); % standing configuration with left arm out to side
end

kinsol = r.doKinematics(q_end);

% yellow: hand
hand_xyz_quat = r.forwardKin(kinsol,hand,[0,0,0]',fk_opts);
drawFrame(hand_xyz_quat,'arms spread - hand fk', 0.1, [1,1,0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% check constraints here




if options.visualize
  v.draw(0,q_start); % visualize robot with hand in pile
end
% add break point here

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'origin');
lcmgl.glTranslated(0,0,0);
lcmgl.glRotated(0, 0,0,0);
lcmgl.glDrawAxes();
lcmgl.switchBuffers();


kinsol_start = r.doKinematics(q_start);
start_xyz_quat_fk = r.forwardKin(kinsol_start,hand,hand_to_palm_offset,2);
hand_xyz_quat_fk = r.forwardKin(kinsol_start,hand,[0,0,0]',fk_opts); % yellow: start left hand position:
%r_hand_xyz_quat = r.forwardKin(kinsol,r_hand,[0,0,0]',fk_opts);
drawFrame(start_xyz_quat_fk,'in the pile - point fk start', 0.1,[1,0,0]); % red: start pose of lower end of capsule
drawFrame(hand_xyz_quat_fk, 'in the pile - hand fk start', 0.1, [1,1,0]);
drawFrame(start_xyz_quat,'in the pile - hand constraint start', 0.1,[1,0,1]); % purple: start pose of lower end of capsule - constraint
%drawFrame(r_hand_xyz_quat, lcmgl);

% green: end pose of lower end of capsule
kinsol_end = r.doKinematics(q_end);
hand_goal_fk = r.forwardKin(kinsol_end,hand,[0,0,0]',fk_opts); % yellow: start left hand position:
drawFrame(hand_goal_fk,'in the pile - hand fk goal',0.1,[1,0.5,0]);
xyz_quat_goal = r.forwardKin(kinsol_end,hand,hand_to_palm_offset,2);
drawFrame(xyz_quat_goal,'in the pile - point fk goal',0.1,[0,1,0]);

% blue: goal_frame 
drawFrame(goal_xyz_quat,'in the pile - point constraint goal', 0.1,[0,0,1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% check constraints here2

position_constraint_7 = WorldPositionConstraint(r, hand, hand_to_palm_offset, start_xyz_quat_fk(1:3), start_xyz_quat_fk(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, hand, start_xyz_quat_fk(4:7), 0, [1.0, 1.0]);

active_constraints = [base_constraints,{position_constraint_7,quat_constraint_8}];

[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints{:}, ikoptions);


if options.visualize
  v.draw(0,q_start); % visualize robot with hand in pile - 2nd
end
% add break point here

x_start = [start_xyz_quat_fk;q_start];
x_goal = [xyz_quat_goal;q_end];
xyz_box_edge_length = 2;
xyz_min = min(start_xyz_quat_fk(1:3),xyz_quat_goal(1:3)) - xyz_box_edge_length/2;
xyz_max = max(start_xyz_quat_fk(1:3),xyz_quat_goal(1:3)) + xyz_box_edge_length/2;

TA = TA.setTranslationSamplingBounds(xyz_min, xyz_max);
TA = TA.addKinematicConstraint(base_constraints{:});
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
display('About to plan ...')
switch options.planning_mode
  case 'rrt'
    [TA, path_ids_A, info] = TA.rrt(x_start, x_goal, TA, options);
  case 'rrt_connect'
    display('Running RRTConnect')
    [TA, path_ids_A, info, TB] = TA.rrtConnect(x_start, x_goal, TB, options);
end
rrt_time = toc(rrt_timer);
fprintf('Timing:\n');
fprintf('  RRT:       %5.2f s\n', rrt_time);

T_smooth = TA;
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
  fprintf('  Smoothing: %5.2f s\n', smoothing_time);
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

xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
xtraj = xtraj.setOutputFrame(r.getStateFrame());

% to view the trajectory:
v.playback(xtraj);
end
