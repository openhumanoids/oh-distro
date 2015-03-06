function replaySim(simVars)

%Unpack simVars
names = fieldnames(simVars);
for i=1:length(names)
n = names{i};
eval([n '= simVars.(names{i});'])
end

% TIMEOUT 600
if nargin < 5 || isempty(options), options = struct(); end
if nargin < 6, rng; else rng(rng_seed); end
w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
if ~isfield(options,'scene'), options.scene = 'scene1'; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
r = RigidBodyManipulator(urdf,options);
nq = r.getNumPositions();
S = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);
q_nom = S.xstar(1:nq);
q_zero = zeros(nq, 1);

world = r.findLinkId('world');
l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
l_hand = r.findLinkId('l_hand');
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
joints = Point(r.getStateFrame, (1:r.getStateFrame.dim)');

% Add obstacles

switch options.scene
    case 'debris'
        collision_object = RigidBodyCapsule(0.05,1,[0.95,0.22,0.35],[0,pi/2,0]);
        collision_object.c = [0.5;0.4;0.3];
        r = addGeometryToBody(r, world, collision_object);

        collision_object = RigidBodyCapsule(0.05,1,[0.95,-0.22,0.35],[0,pi/2,0]);
        collision_object.c = [0.5;0.4;0.3];
        r = addGeometryToBody(r, world, collision_object);

        collision_object = RigidBodyCapsule(0.05,1,[0.8,-0.05,0.35],[-pi/4,0,0]);
        collision_object.c = [0.5;0.4;0.3];
        r = addGeometryToBody(r, world, collision_object);

        collision_object = RigidBodyCapsule(0.05,1,[0.45,-0.05,0.35],[-pi/4,0,0]);
        collision_object.c = [0.5;0.4;0.3];
        r = addGeometryToBody(r, world, collision_object);

        collision_object = RigidBodyCapsule(0.05,1, [0.35,0.27,0],[0,pi/2,0]);
        collision_object.c = [0.5;0.4;0.3];
        r = addGeometryToBody(r, l_hand, collision_object);
        collision_object = RigidBodyCapsule(0.05,1,[0;0;0],[0,pi/2,0]);
        collision_object.c = [0.5;0.4;0.3];
    case 'scene1'
        table = RigidBodyBox([1 1 .025], [.9 0 .9], [0 0 0]);
        r = addGeometryToBody(r, world, table);
        
        targetObjectPos = [0.7 0 1.05];
        targetObject = RigidBodyBox([.05 .05 .3], targetObjectPos, [0 0 0]);
        r = addGeometryToBody(r, world, targetObject);
end

r = r.compile();
warning(w);

xyz_quat_start = [0.5969; -0.1587; 0.9; -0.2139; 0.6724; 0.3071; -0.6387];

% IK constraints

%Quasi Static Constraint
qsc_constraint_0 = QuasiStaticConstraint(r, [-inf, inf], 1);
qsc_constraint_0 = qsc_constraint_0.setShrinkFactor(0.5);
qsc_constraint_0 = qsc_constraint_0.setActive(true);
qsc_constraint_0 = qsc_constraint_0.addContact(l_foot, l_foot_pts);
qsc_constraint_0 = qsc_constraint_0.addContact(r_foot, r_foot_pts);

%Left Foot Constraints
point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999962214379723, 3.8873668451910772e-05, 0.00086844752325226373, -0.024113362129690341; -4.319650228383918e-05, 0.99998760778828055, 0.0049781928381826216, 0.13142880655433892; -0.00086824324064880729, -0.0049782284710370005, 0.99998723161596681, 0.081845132612297311; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_1 = WorldPositionConstraint(r, l_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_2 = WorldQuatConstraint(r, l_foot, [0.99999680768841015; -0.0024891132733300568; 0.00043417407699420605; -2.0517608182535892e-05], 0.0, [0.0, 1.0]);

%Right Foot Constraints
point_in_link_frame = [0.0; 0.0; 0.0];
ref_frame = [0.99999972333813658, -3.8603987442147522e-05, 0.00074285488657430923, -0.024113358389590833; 4.2294235092508014e-05, 0.99998765711726534, -0.0049682818277853539, -0.13142881299268941; -0.00074265392211426647, 0.0049683118717304582, 0.99998738209154281, 0.081845129013906948; 0.0, 0.0, 0.0, 1.0];
lower_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
upper_bounds = [0.0; 0.0; 0.0] + [0.0; 0.0; 0.0];
ref_frame = inv(ref_frame);
position_constraint_3 = WorldPositionConstraint(r, r_foot, ref_frame(1:3,:)*[point_in_link_frame;1], lower_bounds, upper_bounds, [0.0, 1.0]);


quat_constraint_4 = WorldQuatConstraint(r, r_foot, [0.99999684531339206; 0.0024841562616134435; 0.00037137837375452614; 2.0224619435999976e-05], 0.0, [0.0, 1.0]);

%Back Constraints
posture_constraint_5 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.back_bkx; joints.back_bky; joints.back_bkz];
joints_lower_limit = q_zero(joint_inds) + [-0.08726646259971647; -0.08726646259971647; -inf];
joints_upper_limit = q_zero(joint_inds) + [0.08726646259971647; 0.08726646259971647; inf];
posture_constraint_5 = posture_constraint_5.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

%Base Constraints
posture_constraint_6 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.base_x; joints.base_y; joints.base_roll; joints.base_pitch; joints.base_yaw];
joints_lower_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
joints_upper_limit = q_nom(joint_inds) + [0.0; 0.0; 0.0; 0.0; 0.0];
posture_constraint_6 = posture_constraint_6.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

% fixed right arm
posture_constraint_7 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.r_arm_usy; joints.r_arm_shx; joints.r_arm_ely; joints.r_arm_elx; joints.r_arm_uwy; joints.r_arm_mwx; joints.neck_ay];
joints_lower_limit = q_nom(joint_inds);
joints_upper_limit = q_nom(joint_inds);
posture_constraint_7 = posture_constraint_7.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

posture_constraint_8 = PostureConstraint(r, [-inf, inf]);
joint_inds = [joints.l_leg_kny;joints.r_leg_kny];
joints_lower_limit = 30*pi/180*[1;1];
joints_upper_limit = 120*pi/180*[1;1];
posture_constraint_8 = posture_constraint_8.setJointLimits(joint_inds, joints_lower_limit, joints_upper_limit);

v = r.constructVisualizer();

drawTree(TA);
drawTree(TB);
drawPath(T_smooth, path_ids_A);

q_path = extractPath(T_smooth, path_ids_A);
path_length = size(q_path,2);

q_idx = TA.idx{TA.cspace_idx};

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
v.playback(xtraj);

end


