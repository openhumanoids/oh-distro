function atlasKneeSingularityTest()
% NOTEST
% Make sure we can approach and leave the knee singularity gracefully by commanding a set of poses which are unreachable for the leg. 

checkDependency('iris');
checkDependency('lcmgl');

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('atlas_version', 5,...
                                                    'floating', true,...
                                                    'use_new_kinsol', true,...
                                                    'hand_right', 'robotiq_weight_only',...
                                                    'hand_left', 'robotiq_weight_only'));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = DRCAtlas();
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

lc = lcm.lcm.LCM.getSingleton();
state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
lc.subscribe('EST_ROBOT_STATE', state_monitor);
data = [];
while isempty(data)
  data = state_monitor.getNextMessage(10);
end
[x0, ~] = r.getStateFrame().lcmcoder.decode(data);

nq = r.getNumPositions();

kinsol = r.doKinematics(x0(1:nq));
rfoot_pose = r.forwardKin(kinsol, r.foot_frame_id.right, [0;0;0], 2);
com0 = r.getCOM(kinsol);
T_orig = [quat2rotmat(rfoot_pose(4:7)), rfoot_pose(1:3); 0,0,0,1];
lfoot_pose = r.forwardKin(kinsol, r.foot_frame_id.left, [0;0;0], 2);

lfoot_poses = [lfoot_pose,...
               lfoot_pose,...
               lfoot_pose + [0;0;0.05;0;0;0;0],...
               transformPose([1;0.26;0.2;expmap2quat([0;-pi/2;0])], T_orig),...
               transformPose([0;0.26;0.2;1;0;0;0], T_orig),...
               transformPose([-1;0.26;0.2;expmap2quat([0;pi/2;0])], T_orig),...
               transformPose([0;0.26;0.2;1;0;0;0], T_orig),...
               lfoot_pose + [0;0;0.05;0;0;0;0],...
               lfoot_pose,...
               lfoot_pose,...
               lfoot_pose,...
               lfoot_pose];

dt = 10;
foot_ts = [0, 4, 8, 8+dt*(1:(size(lfoot_poses,2)-3))];
nposes = length(foot_ts);
lfoot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.left, foot_ts, lfoot_poses);
lfoot_motion.in_floating_base_nullspace = true(1,nposes);
lfoot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

rfoot_poses = repmat(rfoot_pose, 1, nposes);
rfoot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.right, foot_ts, rfoot_poses);
rfoot_motion.in_floating_base_nullspace = true(1,nposes);
rfoot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];


double_support = RigidBodySupportState(r, [r.foot_body_id.left, r.foot_body_id.right]);
zmp_knots = struct('t', num2cell(foot_ts([1,2,end-3:end])),...
                   'zmp', {com0(1:2), rfoot_pose(1:2), rfoot_pose(1:2), rfoot_pose(1:2), com0(1:2), com0(1:2)},...
                   'supp', {double_support, r.right_full_support, r.right_full_support, double_support, double_support, double_support});

plan = QPLocomotionPlan.from_biped_foot_and_zmp_knots([rfoot_motion, lfoot_motion], zmp_knots, r, x0);
plan.gain_set = 'walking';

lc.publish('CONFIGURATION_TRAJ', DRCQPLocomotionPlan.toLCM(plan));

end

function pose = transformPose(pose_quat, T)
  xyz = T * [pose_quat(1:3); 1];
  xyz = xyz(1:3);
  quat = rotmat2quat(T(1:3,1:3) * quat2rotmat(pose_quat(4:7)));
  pose = [xyz; quat];
end
