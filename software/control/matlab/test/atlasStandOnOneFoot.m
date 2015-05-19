function atlasStandOnOneFoot(stance_foot)
% NOTEST

if strcmp(stance_foot, 'right')
  swing_foot = 'left';
elseif strcmp(stance_foot, 'left')
  swing_foot = 'right';
else
  error('invalid foot: %s', stance_foot)
end

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
for f = {'right', 'left'}
  foot = f{1};
  foot_pose.(foot) = r.forwardKin(kinsol, r.foot_frame_id.(foot), [0;0;0], 2);
end
com0 = r.getCOM(kinsol);

swing_foot_poses = [foot_pose.(swing_foot),...
                    foot_pose.(swing_foot),...
                    foot_pose.(swing_foot) + [0;0;0.05;0;0;0;0]];
foot_ts = [0, 1, 2];
nposes = length(foot_ts);
swing_foot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.(swing_foot), foot_ts, swing_foot_poses);
swing_foot_motion.in_floating_base_nullspace = true(1, nposes);
swing_foot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

stance_foot_poses = repmat(foot_pose.(stance_foot), 1, nposes);
stance_foot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.(stance_foot), foot_ts, stance_foot_poses);
stance_foot_motion.in_floating_base_nullspace = true(1, nposes);
stance_foot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

double_support = RigidBodySupportState(r, [r.foot_body_id.left, r.foot_body_id.right], {{'heel','toe'}, {'heel','toe'}});
single_support = RigidBodySupportState(r, [r.foot_body_id.(stance_foot)], {{'heel', 'toe'}})
zmp_knots = struct('t', num2cell(foot_ts([1,2,3])),...
                   'zmp', {com0(1:2), foot_pose.(stance_foot)(1:2), foot_pose.(stance_foot)(1:2)},...
                   'supp', {double_support, single_support, single_support});

plan_settings = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots([swing_foot_motion, stance_foot_motion], zmp_knots, r, x0);
plan_settings.gain_set = 'walking';

lc.publish('CONFIGURATION_TRAJ', DRCQPLocomotionPlan.toLCM(plan_settings));


disp('press enter to return to double support')
pause()

swing_foot_poses = [foot_pose.(swing_foot) + [0;0;0.05;0;0;0;0],...
                    foot_pose.(swing_foot),...
                    foot_pose.(swing_foot),...
                    foot_pose.(swing_foot)];
foot_ts = [0, 1, 2, 3];
nposes = length(foot_ts);
swing_foot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.(swing_foot), foot_ts, swing_foot_poses);
swing_foot_motion.in_floating_base_nullspace = true(1, nposes);
swing_foot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

stance_foot_poses = repmat(foot_pose.(stance_foot), 1, nposes);
stance_foot_motion = BodyMotionData.from_body_xyzquat(r.foot_frame_id.(stance_foot), foot_ts, stance_foot_poses);
stance_foot_motion.in_floating_base_nullspace = true(1, nposes);
stance_foot_motion.weight_multiplier = [0.1;0.1;0.1;1;1;1];

double_support = RigidBodySupportState(r, [r.foot_body_id.left, r.foot_body_id.right], {{'heel','toe'}, {'heel','toe'}});
single_support = RigidBodySupportState(r, [r.foot_body_id.(stance_foot)], {{'heel', 'toe'}})
zmp_knots = struct('t', num2cell(foot_ts([1,2,3,4])),...
                   'zmp', {foot_pose.(stance_foot)(1:2), foot_pose.(stance_foot)(1:2), com0(1:2), com0(1:2)},...
                   'supp', {single_support, single_support, double_support, double_support});

plan_settings = QPLocomotionPlanSettings.fromBipedFootAndZMPKnots([swing_foot_motion, stance_foot_motion], zmp_knots, r, x0);
plan_settings.gain_set = 'walking';

lc.publish('CONFIGURATION_TRAJ', DRCQPLocomotionPlan.toLCM(plan_settings));

end
