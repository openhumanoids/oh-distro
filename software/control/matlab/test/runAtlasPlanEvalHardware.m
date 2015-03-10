function runAtlasPlanEvalHardware(controller_type, run_in_simul_mode,atlas_version)

if nargin < 1
  controller_type = 2; % 1: PID, 2: PID+manip params, 3: PD+gravity comp, 4: inverse dynamics
  run_in_simul_mode = 0; % Initialize to support drakeWalking-like controllers
                         % (is this redundant with controller_type?)
                         % Use value 1 for normal, 2 to add weights to the
                         % control model's hands
end
if nargin < 2
  run_in_simul_mode = 0;
end
if nargin < 3
  atlas_version = 4;
end

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

% % x0 = [];
% % while isempty(x0)
% %   [x0, ~] = state_frame.getNextMessage(50);
% % end

% % set initial state to fixed point
% load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
% xstar = r.resolveConstraints(xstar);
% r = r.setInitialState(xstar);
% x0 = xstar;

% nq = getNumPositions(r);

% navgoal = example_options.navgoal + [x0(1:2); 0;0;0;0];
% R=rotz(navgoal(6));

% rfoot_navgoal = navgoal;
% lfoot_navgoal = navgoal;

% rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
% lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% % Plan footsteps to the goal
% goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
% footstep_plan = r.planFootsteps(x0(1:nq), goal_pos, [], struct('step_params', struct('max_num_steps', example_options.num_steps)));

% walking_plan = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);


state_msg_data = [];
while isempty(state_msg_data)
  state_msg_data = state_monitor.getNextMessage(10);
end
[x0, t0] = state_coder.decode(drc.robot_state_t(state_msg_data));

standing_plan = StandingPlan.from_standing_state(x0, r);

% standing_plan.duration = 3;
% queue = {standing_plan, walking_plan};

queue = {standing_plan};
planeval = DRCPlanEval(r, 'hardware', queue);

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
