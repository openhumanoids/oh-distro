function runAtlasPlanEvalAsync(example_options)
%NOTEST 

checkDependency('gurobi');
checkDependency('lcmgl');

if nargin<1, example_options=struct(); end
example_options = applyDefaults(example_options, struct('use_mex', true,...
                                                        'use_bullet', false,...
                                                        'navgoal', [0.5;0;0;0;0;0],...
                                                        'quiet', true,...
                                                        'num_steps', 4,...
                                                        'terrain', RigidBodyFlatTerrain));

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
options.use_bullet = example_options.use_bullet;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
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
