urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');

options.floating = true;
options.terrain = MyRigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;

r = RigidBodyManipulator(urdf, options);
v = r.constructVisualizer();
fp = load([getDrakePath(), '/../control/matlab/data/valkyrie/valkyrie_fp.mat']);
x = fp.xstar(1:r.getNumPositions());
v.draw(0,x)
q_path = x;
  
%Set IK options
ik_seed_pose = x;
ik_nominal_pose = x;
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

l_foot = r.findLinkId('LeftUpperFoot');
r_foot = r.findLinkId('RightUpperFoot');

l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
quasiStaticConstraint = QuasiStaticConstraint(r, [-inf, inf], 1);
quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
quasiStaticConstraint = quasiStaticConstraint.setActive(true);
quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);

kinsol = r.doKinematics(ik_nominal_pose);
footPose = r.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
leftFootPosConstraint = WorldPositionConstraint(r, l_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
leftFootQuatConstraint = WorldQuatConstraint(r, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);
footPose = r.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
rightFootPosConstraint = WorldPositionConstraint(r, r_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
rightFootQuatConstraint = WorldQuatConstraint(r, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);

lHand = r.findLinkId('LeftPalm');
position = [0.3; .8; 1.4];
lhandConstraint = WorldPositionConstraint(r, lHand, [0; 0; 0], position, position);

rHand = r.findLinkId('RightPalm');
position = [0.3; -.8; 1.4];
rhandConstraint = WorldPositionConstraint(r, rHand, [0; 0; 0], position, position);

constraints = {lhandConstraint, rhandConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,  rightFootQuatConstraint};

[q, info, infeasibleConstraints] = inverseKin(r, ik_seed_pose, ik_nominal_pose, constraints{:}, ikoptions);
v.draw(0,q)

q_path = [q_path q];


position = [0.1; .8; .9];
lhandConstraint = WorldPositionConstraint(r, lHand, [0; 0; 0], position, position);

position = [0.1; -.8; .9];
rhandConstraint = WorldPositionConstraint(r, rHand, [0; 0; 0], position, position);

kinsol = r.doKinematics(ik_nominal_pose);
pelvis = r.findLinkId('pelvis');
pelvisPose = r.forwardKin(kinsol,pelvis, [0; 0; 0], 2);
pelvisConstraint = WorldPositionConstraint(r, pelvis, [0; 0; 0], pelvisPose(1:3) + [-1; -1; -.2], pelvisPose(1:3) + [1; 1; -.2]);

constraints = {quasiStaticConstraint, lhandConstraint, rhandConstraint, leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint,  rightFootQuatConstraint};

[q, info, infeasibleConstraints] = inverseKin(r, ik_seed_pose, ik_nominal_pose, constraints{:}, ikoptions);
v.draw(0,q)

q_path = [q_path q];
q_path = [q_path ik_nominal_pose];

path_length = size(q_path,2);

% Scale timing to obey joint velocity limits
% Create initial spline
q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path));
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

v.playback(xtraj);