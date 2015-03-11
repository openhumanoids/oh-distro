function runAtlasPlanEvalStandalone(run_in_simul_mode,atlas_version,walking_options)

if nargin < 1
  run_in_simul_mode = 0;
end
if nargin < 2
  atlas_version = 4;
end
if nargin < 3
  walking_options = struct();
end

walking_options = applyDefaults(walking_options, struct('navgoal', [0.5;0;0;0;0;0],...
                                                        'num_steps', 4));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
options.floating = true;
options.ignore_friction = true;
options.run_in_simul_mode = run_in_simul_mode;
options.atlas_version = atlas_version;
if (run_in_simul_mode == 2)
  options.hands = 'robotiq_weight_only';
end

r = DRCAtlas([],options);
r = setTerrain(r,DRCTerrainMap(true,struct('name','Controller','listen_for_foot_pose',false)));
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

state_coder = drcFrames.AtlasState(r).lcmcoder;
state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
lc = lcm.lcm.LCM.getSingleton();
lc.subscribe('EST_ROBOT_STATE', state_monitor);

state_msg_data = [];
while isempty(state_msg_data)
  state_msg_data = state_monitor.getNextMessage(50);
end
[x0, ~] = state_coder.decode(drc.robot_state_t(state_msg_data));

% % set initial state to fixed point
% load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
% xstar = r.resolveConstraints(xstar);
% r = r.setInitialState(xstar);
% x0 = xstar;

nq = getNumPositions(r);

navgoal = walking_options.navgoal + [x0(1:2); 0;0;0;0];
R=rotz(navgoal(6));

rfoot_navgoal = navgoal;
lfoot_navgoal = navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', struct('max_num_steps', walking_options.num_steps)));

walking_plan = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);

standing_plan = StandingPlan.from_standing_state(x0, r);

standing_plan.duration = 3;
% queue = {standing_plan, walking_plan};

queue = {standing_plan};
if options.run_in_simul_mode
  mode = 'sim';
else
  mode = 'hardware';
end
planeval = DRCPlanEval(r, mode, queue);

disp('plan eval ready');
while true
  state_msg_data = state_monitor.getNextMessage(10);
  if isempty(state_msg_data)
    continue
  end
  [x, t] = state_coder.decode(drc.robot_state_t(state_msg_data));
  qp_input = planeval.getQPControllerInput(t, x);
  encodeQPInputLCMMex(qp_input);
end
