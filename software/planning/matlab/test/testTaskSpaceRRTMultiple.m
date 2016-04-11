function [xtraj,info,v] = testTaskSpaceRRTMultiple(model, scene, options, rng_seed)
% matlab -nosplash -nodesktop -r "addpath(fullfile(getenv('DRC_BASE'),'software/ipab/control/matlab/tests')); addpath_control; testTaskSpaceRRTMultiple(); "
% multi-robot test script for combinations of humanoids
%
% model: v3,v4,v and val1
% scene = 1 % 1. reach around an capsule   2. reach to object on a table through clutter
%
% supported combos:
% v3,v5 and val1, val2 with scene 1
% val1, val2 with scene 2

% issues:
% forearm and head need to be disabled (unnecessarily) for val1
% r_arm and neck joints are lumped together
% scene 2 for atlas robots dont work as r_arm intersects - need to fix it
% ik is solved 3 times. I think its resolved for q_start twice
%
% DEBUG tip:
% insert this snippet into collisionDetect.m (L84) to print the colliding links:
%      valid = all(phi > 0.009);
%      if (valid == 0)
%        if min(phi>0)
%          [a,b] = min(phi) ;  obj.body( idxA(b)).linkname , obj.body( idxB(b) ).linkname
%          keyboard
%        end
%      end

% TIMEOUT 600
if nargin < 1; model = 'val2'; end
if nargin < 2; scene = 2; end
if nargin < 3 || isempty(options), options = struct(); end
if nargin < 4, rng; else rng(rng_seed); end
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

options.l_foot_link_name = 'l_foot';
options.r_foot_link_name = 'r_foot';
options.l_hand_link_name = 'l_hand';
options.r_hand_link_name = 'r_hand';

if strcmp(model,'v3')
  urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
  S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
%elseif (which_robot == 4) % version 4 is not supported 
%  urdf = fullfile(getDrakePath(),'..','models','atlas_v4','model_convex_hull_fingers.urdf');
%  S = load([getDrakePath(), '/../control/matlab/data/atlas_v4/atlas_v4_fp.mat']);
elseif strcmp(model,'v5')
  urdf = fullfile(getDrakePath(),'..','models','atlas_v5','model_convex_hull_fingers.urdf');
  S = load([getDrakePath(), '/../control/matlab/data/atlas_v5/atlas_v5_fp.mat']);
elseif strcmp(model,'val1')
  urdf = fullfile(getDrakePath(),'../..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
  S = load([getDrakePath(), '/../control/matlab/data/valkyrie/valkyrie_fp.mat']);
  options.l_foot_link_name = 'LeftUpperFoot';
  options.r_foot_link_name = 'RightUpperFoot';
  options.l_hand_link_name = 'LeftPalm';
  options.r_hand_link_name = 'RightPalm';  
elseif strcmp(model,'val2')
  urdf = fullfile(getDrakePath(),'../../models/val_description/urdf/valkyrie_sim_drake.urdf');
  S = load([getDrakePath(), '/../../control/matlab/data/val_description/valkyrie_fp_june2015.mat']);
  options.l_foot_link_name = 'leftFoot';
  options.r_foot_link_name = 'rightFoot';
  options.l_hand_link_name = 'leftPalm';
  options.r_hand_link_name = 'rightPalm';
end

r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

world = r.findLinkId('world');
l_foot = r.findLinkId(options.l_foot_link_name);
r_foot = r.findLinkId(options.r_foot_link_name);
l_hand = r.findLinkId(options.l_hand_link_name);
r_hand = r.findLinkId(options.r_hand_link_name);

r = create_scene(r, scene, world);

% reach the hand from:
if strcmp(model,'v3') && (scene == 1)
  start_xyz_quat = [0.2;0.75;1.2; rpy2quat([90*pi/180, 1.5789, 0]) ];
  goal_xyz_quat = [0.5;0.3;1.2; rpy2quat([90*pi/180, 1.5789, 0]) ];
elseif strcmp(model,'v5') && (scene == 1)
  start_xyz_quat = [0.2;0.75;1.2; rpy2quat([90*pi/180, -1.5789, 0]) ];
  goal_xyz_quat = [0.5;0.3;1.2; rpy2quat([90*pi/180, -1.5789, 0]) ];
elseif strcmp(model,'val1')
  if (scene == 1)
    start_xyz_quat = [0.3;0.6;0.9; rpy2quat([-90*pi/180,0*pi/180,0*pi/180] ) ];
    goal_xyz_quat = [0.4;0.4;1.1; rpy2quat([-90*pi/180,0*pi/180,0*pi/180] ) ];
  elseif (scene == 2)
    start_xyz_quat = [0.05;0.4;0.8; rpy2quat([-90*pi/180,60*pi/180,0*pi/180] ) ];
    goal_xyz_quat = [0.4;0.3;1.1; rpy2quat([-90*pi/180,0*pi/180,0*pi/180] ) ];
  end
elseif strcmp(model,'val2')
  if (scene == 1)
    start_xyz_quat = [0.3;0.6;0.9; rpy2quat([0*pi/180,0*pi/180,-90*pi/180] ) ];
    goal_xyz_quat = [0.4;0.4;1.1; rpy2quat([0*pi/180,0*pi/180,-90*pi/180] ) ];
  elseif (scene == 2)
    start_xyz_quat = [0.05;0.4;0.8; rpy2quat([-60*pi/180,0*pi/180,-90*pi/180] ) ];
    goal_xyz_quat = [0.4;0.3;1.1; rpy2quat([0*pi/180,0*pi/180,-90*pi/180] ) ];
  end
else
  disp('combination not implemented')
  keyboard
end

%%% Configuration (scene independent) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (strcmp(model,'v3') || strcmp(model,'v5'))
  ref_frame_l_foot = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341; -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892; -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311; 0.0, 0.0, 0.0, 1.0];
  ref_frame_r_foot = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833; 4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941; -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948; 0.0, 0.0, 0.0, 1.0];

  joint_inds_back = [joints.back_bkx; joints.back_bky; joints.back_bkz];
  joint_inds_base = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
  
  joint_inds_knees = [joints.l_leg_kny;joints.r_leg_kny];
  joints_lower_limit_knees = 30*pi/180*[1;1];
  joints_upper_limit_knees = 120*pi/180*[1;1];

  if strcmp(model,'v3')
    hand_to_palm_offset = [0; 0.24449999999999988; 0.011200000000000071];
    joint_inds_r_arm = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
  elseif strcmp(model,'v5')
    hand_to_palm_offset = [0; -0.24449999999999988; 0.011200000000000071];
    joint_inds_r_arm = [joints.r_arm_shz; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.r_arm_lwy; joints.neck_ay];
  end
  
  inactive_collision_bodies = [l_foot, r_foot];
elseif strcmp(model,'val1')
  ref_frame_l_foot = [1,0,0,-0.068; 0,1,0,0.139; 0,0,1,0.085; 0,0,0,1];  ref_frame_l_foot(1:3,1:3) = rpy2rotmat([0,97.4*pi/180,180*pi/180]);
  ref_frame_r_foot = [1,0,0,-0.068; 0,1,0,-0.139; 0,0,1,0.085; 0,0,0,1];   ref_frame_r_foot(1:3,1:3) = rpy2rotmat([0,82.6*pi/180,0*pi/180]);

  joint_inds_back = [joints.WaistLateralExtensor; joints.WaistExtensor; joints.WaistRotator];
  joint_inds_base = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
  joint_inds_r_arm = [joints.RightShoulderExtensor; joints.RightShoulderAdductor; joints.RightShoulderSupinator; joints.RightElbowExtensor; joints.RightForearmSupinator; joints.RightWristExtensor; joints.RightWrist; joints.LowerNeckExtensor; joints.NeckRotator; joints.UpperNeckExtensor];
  
  joint_inds_knees = [joints.LeftKneeExtensor; joints.RightKneeExtensor];
  joints_lower_limit_knees = -1.9*[1;1];
  joints_upper_limit_knees = -0.6*[1;1];  
  
  hand_to_palm_offset = [0.06; 0.0; -0.02];

  % internal parts removed as they created self collisions:
  LeftHipRotator = r.findLinkId('LeftHipRotator');
  RightHipRotator = r.findLinkId('RightHipRotator');
  LeftHipAdductor = r.findLinkId('LeftHipAdductor');
  RightHipAdductor = r.findLinkId('RightHipAdductor');
  LowerNeckExtensor = r.findLinkId('LowerNeckExtensor');

  % these shouldn't be culled from the link list but its easier too do than
  % fixing meshes now:
  RightForearm = r.findLinkId('RightForearm'); % main welding link
  LeftForearm = r.findLinkId('LeftForearm'); % main welding link
  Head = r.findLinkId('Head'); % main welding link

  inactive_collision_bodies = [l_foot,r_foot, LeftHipAdductor, RightHipAdductor,  LeftHipRotator, RightHipRotator, LowerNeckExtensor, LeftForearm, RightForearm, Head];
elseif strcmp(model,'val2')
  %ref_frame_l_foot = [1,0,0,-0.068; 0,1,0,0.139; 0,0,1,0.085; 0,0,0,1];  ref_frame_l_foot(1:3,1:3) = rpy2rotmat([0,97.4*pi/180,180*pi/180]);
  ref_frame_l_foot = [0.9921, 0, -0.1256, -0.0698; 0, 1, 0   0.1377; 0.1256, 0, 0.9921, 0.0862; 0, 0, 0, 1];
  ref_frame_r_foot = [0.9921, 0, -0.1256, -0.0698; 0, 1, 0, -0.1377; 0.1256, 0, 0.9921, 0.0862; 0, 0, 0, 1];

  joint_inds_back = [joints.torsoYaw; joints.torsoPitch; joints.torsoRoll];
  joint_inds_base = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
  joint_inds_r_arm = [joints.rightShoulderPitch; joints.rightShoulderRoll; joints.rightShoulderYaw; joints.rightElbowPitch; joints.rightForearmYaw; joints.rightWristRoll; joints.rightWristPitch];
  %; joints.LowerNeckPitch; joints.NeckYaw; joints.UpperNeckPitch];

  joint_inds_knees = [joints.leftKneePitch; joints.rightKneePitch];
  joints_lower_limit_knees = 0.6*[1;1];
  joints_upper_limit_knees = 1.9*[1;1];

  hand_to_palm_offset = [0.0; 0.0; -0.02];% [0.06; 0.0; -0.02];

  % internal parts removed as they created self collisions:
  leftHipYawLink = r.findLinkId('leftHipYawLink');
  leftHipRollLink = r.findLinkId('leftHipRollLink');
  rightHipYawLink = r.findLinkId('rightHipYawLink');
  rightHipRollLink = r.findLinkId('rightHipRollLink');
  upperNeckPitchLink = r.findLinkId('upperNeckPitchLink');
  torsoPitchLink = r.findLinkId('torsoPitchLink'); % main welding link
  torsoYawLink = r.findLinkId('torsoYawLink'); % main welding link
  rightShoulderYawLink = r.findLinkId('rightShoulderYawLink');
  leftShoulderYawLink = r.findLinkId('leftShoulderYawLink');
  
  % these shouldn't be culled from the link list but its easier too do than
  % fixing meshes now:
  rightForearmLink = r.findLinkId('rightForearmLink'); % main welding link
  leftForearmLink = r.findLinkId('leftForearmLink'); % main welding link
  head = r.findLinkId('head'); % main welding link

  inactive_collision_bodies = [l_foot,r_foot, torsoPitchLink, torsoYawLink, leftHipYawLink, leftHipRollLink,  rightHipYawLink, rightHipRollLink, upperNeckPitchLink, leftForearmLink, rightForearmLink, head, rightShoulderYawLink, leftShoulderYawLink];
end



%%%%%% All the stuff above is set up %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r = r.compile();
warning(w);


% IK constraints
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);

% fixed points on feet
point_in_link_frame_l_foot = [0.0; 0.0; 0.0];
point_in_link_frame_r_foot = [0.0; 0.0; 0.0];

l_foot_xyz_quat_ref_frame = frame2xyzquat( ref_frame_l_foot); quat_l_foot = l_foot_xyz_quat_ref_frame(4:7);
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame_l_foot = inv(ref_frame_l_foot);
position_constraint_1 = WorldPositionConstraint(r, l_foot, ref_frame_l_foot(1:3,:)*[point_in_link_frame_l_foot;1], lower_bounds, upper_bounds, [0.0, 1.0]);
quat_constraint_2 = WorldQuatConstraint(r, l_foot, quat_l_foot, 0.0, [0.0, 1.0]);

r_foot_xyz_quat_ref_frame = frame2xyzquat( ref_frame_r_foot); quat_r_foot = r_foot_xyz_quat_ref_frame(4:7);
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame_r_foot = inv(ref_frame_r_foot);
position_constraint_3 = WorldPositionConstraint(r, r_foot, ref_frame_r_foot(1:3,:)*[point_in_link_frame_r_foot;1], lower_bounds, upper_bounds, [0.0, 1.0]);
quat_constraint_4 = WorldQuatConstraint(r, r_foot, quat_r_foot, 0.0, [0.0, 1.0]);

posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joints_lower_limit = q_zero(joint_inds_back) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds_back) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds_back, joints_lower_limit, joints_upper_limit);

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joints_lower_limit = q_nom(joint_inds_base) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_nom(joint_inds_base) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds_base, joints_lower_limit, joints_upper_limit);

% fixed right arm
posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joints_lower_limit = q_nom(joint_inds_r_arm);
joints_upper_limit = q_nom(joint_inds_r_arm);
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds_r_arm, joints_lower_limit, joints_upper_limit);

posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds_knees, joints_lower_limit_knees, joints_upper_limit_knees);

lower_bounds = [0.0; 0.0; 0.0] + [-0; -0; -0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
goal_frame = xyzquat2frame(goal_xyz_quat);
position_constraint_7 = WorldPositionInFrameConstraint(r, l_hand, hand_to_palm_offset, goal_frame, lower_bounds, upper_bounds, [1.0, 1.0]);
%goal_frame = inv(goal_frame);
%position_constraint_7 = WorldPositionConstraint(r, l_hand, goal_frame(1:3,:)*[hand_to_palm_offset;1], lower_bounds, upper_bounds, [1.0, 1.0]);

l_hand_initial_position_constraint = WorldPositionConstraint(r, l_hand, [0;0;0], start_xyz_quat(1:3), start_xyz_quat(1:3));
% offset from hand link around which the planning problem is framed:
% for val this is the center of the palm


temp = frame2xyzquat( goal_frame); quat_8 = temp(4:7);
quat_constraint_8 = WorldQuatConstraint(r, l_hand, quat_8, 0*pi/180, [1.0, 1.0]); % was 10*pi/180 , mfallon

l_hand_initial_quat_constraint = WorldQuatConstraint(r, l_hand, start_xyz_quat(4:7), 0*pi/180);

min_distance = 0.01;
% this seems to disable these joints from being used for collision checking:
active_collision_options.body_idx = setdiff(1:r.getNumBodies(), inactive_collision_bodies);

options.display_after_every = 100;

%TA = TaskSpaceMotionPlanningTree(r, 'l_hand', hand_to_palm_offset);
TA = TaskSpaceMotionPlanningTree(r, options.l_hand_link_name, hand_to_palm_offset);

TA  = TA.setMinDistance(min_distance);
TA  = TA.setOrientationWeight(1);
TA.max_edge_length = 0.05;
TA.max_length_between_constraint_checks = TA.max_edge_length;
TA.angle_tol = 1*pi/180;
TA.position_tol = 1e-3;
% this seems to disable these joints from being used for collision checking:
TA.trees{TA.cspace_idx}.active_collision_options.body_idx = setdiff(1:r.getNumBodies(), inactive_collision_bodies);

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


active_constraints_start = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6, posture_constraint_7,posture_constraint_8, l_hand_initial_position_constraint , l_hand_initial_quat_constraint};
[q_start, info, infeasible_constraint_start] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints_start{:}, ikoptions);

if options.visualize
  v = r.constructVisualizer(); % robot arms spread, pelvise at zero
else
  v = [];
end

posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_start(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

base_constraints = {qsc_constraint_0, position_constraint_1, quat_constraint_2, position_constraint_3, quat_constraint_4, posture_constraint_5,posture_constraint_6,posture_constraint_7,posture_constraint_8};


active_constraints_end = [base_constraints, {position_constraint_7,quat_constraint_8}];
[q_end, info, infeasible_constraint_end] = inverseKin(r, ik_seed_pose, ik_nominal_pose, active_constraints_end{:}, ikoptions);


if options.visualize
  v.draw(0,q_end); % standing configuration with left arm at goal
end

kinsol = r.doKinematics(q_end);

% yellow: l_hand
l_hand_xyz_quat = r.forwardKin(kinsol,l_hand,[0,0,0]',fk_opts);
drawFrame(l_hand_xyz_quat,'arms spread - l hand fk', 0.1, [1,1,0]);

% l_foot ref
drawFrame(l_foot_xyz_quat_ref_frame,'arms spread - l foot constraint', 0.1, [1,0,0]);
% l_foot pose
l_foot_fk = r.forwardKin(kinsol,l_foot,[0,0,0]',fk_opts);
drawFrame(l_foot_fk,'arms spread - l foot fk ',0.1, [1,1,0]);

% r_foot ref
drawFrame(r_foot_xyz_quat_ref_frame, 'arms spread - r foot constraint',0.1, [1,0,0]);
% r_foot pose
r_foot_fk = r.forwardKin(kinsol,r_foot,[0,0,0]',fk_opts);
drawFrame(r_foot_fk, 'arms spread - r foot fk',0.1, [1,1,0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% check constraints here




if options.visualize
  v.draw(0,q_start); % visualize robot by side
end
% add break point here

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'origin');
lcmgl.glTranslated(0,0,0);
lcmgl.glRotated(0, 0,0,0);
lcmgl.glDrawAxes();
lcmgl.switchBuffers();


kinsol_start = r.doKinematics(q_start);
start_xyz_quat_fk = r.forwardKin(kinsol_start,l_hand,hand_to_palm_offset,2);
l_hand_xyz_quat_fk = r.forwardKin(kinsol_start,l_hand,[0,0,0]',fk_opts); % yellow: start left hand position:
%r_hand_xyz_quat = r.forwardKin(kinsol,r_hand,[0,0,0]',fk_opts);
drawFrame(start_xyz_quat_fk,'in the pile - point fk start', 0.1,[1,0,0]); % red: start pose of lower end of capsule
drawFrame(l_hand_xyz_quat_fk, 'in the pile - l hand fk start', 0.1, [1,1,0]);
drawFrame(start_xyz_quat,'in the pile - l hand constraint start', 0.1,[1,0,1]); % purple: start pose of lower end of capsule - constraint
%drawFrame(r_hand_xyz_quat, lcmgl);

% green: end pose of lower end of capsule
kinsol_end = r.doKinematics(q_end);
l_hand_goal_fk = r.forwardKin(kinsol_end,l_hand,[0,0,0]',fk_opts); % yellow: start left hand position:
drawFrame(l_hand_goal_fk,'in the pile - l hand fk goal',0.1,[1,0.5,0]);
xyz_quat_goal = r.forwardKin(kinsol_end,l_hand,hand_to_palm_offset,2);
drawFrame(xyz_quat_goal,'in the pile - point fk goal',0.1,[0,1,0]);

% blue: goal_frame 
drawFrame(goal_xyz_quat,'in the pile - point constraint goal', 0.1,[0,0,1]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% check constraints here2

position_constraint_7 = WorldPositionConstraint(r, l_hand, hand_to_palm_offset, start_xyz_quat_fk(1:3), start_xyz_quat_fk(1:3), [1.0, 1.0]);

quat_constraint_8 = WorldQuatConstraint(r, l_hand, start_xyz_quat_fk(4:7), 0, [1.0, 1.0]);

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


function [r] = create_scene(r, scene, world)
% Add obstacles

if (scene==1)
  % Add obstacles
  collision_object = RigidBodyCapsule(0.05,1,[0.5,0.5,1.0],[0,0,0]);
  collision_object.c = [0.5;0.4;0.3];
  r = addGeometryToBody(r, world, collision_object);
elseif (scene==2)
     table = RigidBodyBox([1 1.1 .025], [.8 0.2 .9], [0 0 0]);
     r = addGeometryToBody(r, world, table);
     
%     obstacle = RigidBodyBox([.3 .3 .4], [.55 .35 1.1125], [0 0 0]);
     obstacle = RigidBodyBox([.3 .3 .4], [.55 .55 1.1125], [0 0 0]);
     r = addGeometryToBody(r, world, obstacle);
     
     
     obstacle = RigidBodyBox([.3 .3 .4], [.55 -0.15 1.1125], [0 0 0]);
     r = addGeometryToBody(r, world, obstacle);
%     
%     obstacle = RigidBodyBox([.3 1 .4], [.55 0 1.5125], [0 0 0]);
%     r = addGeometryToBody(r, world, obstacle);
%     
     targetObject = RigidBodyBox([.05 .05 .3], [0.45 0.22 1.12], [0 0 0]);
     targetObject = targetObject.setColor([1 0 0]);
     r = addGeometryToBody(r, world, targetObject);
end    


end
