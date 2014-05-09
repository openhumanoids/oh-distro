function atlasWalking
%NOTEST
addpath(fullfile(getDrakePath,'examples','ZMP'));

joint_str = {'leg'};% <---- cell array of (sub)strings

% load robot model
r = Atlas();
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

act_idx_map = getActuatedJoints(r);
gains = getAtlasGains(); % change gains in this file

joint_ind = [];
joint_act_ind = [];
for i=1:length(joint_str)
  joint_ind = union(joint_ind,find(~cellfun(@isempty,strfind(state_plus_effort_frame.coordinates(1:nq),joint_str{i}))));
  joint_act_ind = union(joint_act_ind,find(~cellfun(@isempty,strfind(input_frame.coordinates,joint_str{i}))));
end

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,5);

gains_copy = getAtlasGains();
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains_copy.k_f_p(joint_act_ind);
gains.ff_f_d(joint_act_ind) = gains_copy.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains_copy.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains_copy.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = 0;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq);
q0 = x0(1:nq);

% create navgoal
R = rpy2rotmat([0;0;x0(6)]);
v = R*[1;0;0];
navgoal = [x0(1)+v(1);x0(2)+v(2);0;0;0;x0(6)];

% create footstep and ZMP trajectories
footstep_planner = StatelessFootstepPlanner();
request = drc.footstep_plan_request_t();
request.utime = 0;
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.goal_pos = encodePosition3d(navgoal);
request.num_goal_steps = 0;
request.num_existing_steps = 0;
request.params = drc.footstep_plan_params_t();
request.params.max_num_steps = 20;
request.params.min_num_steps = 1;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.28;
request.params.max_step_width = 0.32;
request.params.nom_forward_step = 0.25;
request.params.max_forward_step = 0.30;
request.params.nom_upward_step = 0.2;
request.params.nom_downward_step = 0.2;
request.params.ignore_terrain = false;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_command = 0;
request.params.leading_foot = request.params.LEAD_AUTO;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.2;
request.default_step_params.step_height = 0.05;
request.default_step_params.mu = 1.0;
request.default_step_params.constrain_full_foot_pose = true;

footstep_plan = footstep_planner.plan_footsteps(r, request);

walking_planner = StatelessWalkingPlanner();
request = drc.walking_plan_request_t();
request.initial_state = r.getStateFrame().lcmcoder.encode(0, x0);
request.footstep_plan = footstep_plan.toLCM();
request.use_new_nominal_state = true;
request.new_nominal_state = r.getStateFrame().lcmcoder.encode(0, x0);
walking_plan = walking_planner.plan_walking(r, request, true);
walking_ctrl_data = walking_planner.plan_walking(r, request, false);

% No-op: just make sure we can cleanly encode and decode the plan as LCM
walking_ctrl_data = WalkingControllerData.from_walking_plan_t(walking_ctrl_data.toLCM());

ts = walking_plan.ts;
T = ts(end);

% plot walking traj in drake viewer
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

for i=1:length(ts)
	lcmgl.glColor3f(0, 0, 1);
	lcmgl.sphere([walking_ctrl_data.comtraj.eval(ts(i));0], 0.01, 20, 20);
  lcmgl.glColor3f(0, 1, 0);
	lcmgl.sphere([walking_ctrl_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
end
lcmgl.switchBuffers();

%qtraj = PPTrajectory(foh(ts,walking_plan.xtraj(1:nq,:)));

ctrl_data = SharedDataHandle(struct(...
  'is_time_varying',true,...
  'x0',[walking_ctrl_data.zmptraj.eval(T);0;0],...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',walking_ctrl_data.supports,...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'trans_drift',[0;0;0],...
  'qtraj',q0,...
  'comtraj',walking_ctrl_data.comtraj,...
  'K',walking_ctrl_data.K,...
  'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'neck');findJointIndices(r,'back');findJointIndices(r,'ak')]));

% traj = PPTrajectory(spline(ts,walking_plan.xtraj));
% traj = traj.setOutputFrame(r.getStateFrame);
% v = r.constructVisualizer;
% playback(v,traj,struct('slider',true));

% instantiate QP controller
options.slack_limit = 100;
options.w_qdd = 20.0*ones(nq,1);
options.W_hdot = diag([10;10;10;10;10;10]);
options.w_grf = 0.0075;
options.w_slack = 0.005;
options.Kp = 0; % com-z pd gains
options.Kd = 0; % com-z pd gains
options.input_foot_contacts = true;
options.debug = true;
options.use_mex = true;
options.contact_threshold = 0.0075;
options.output_qdd = true;

qp = MomentumControlBlock(r,{},ctrl_data,options);
vo = VelocityOutputIntegratorBlock(r,options);
fcb = FootContactBlock(r);
fshift = FootstepPlanShiftBlock(r,ctrl_data);

% cascade IK/PD block
options.Kp = 80.0*ones(nq,1);
options.Kd = 20.0*ones(nq,1);
pd = WalkingPDBlock(r,ctrl_data,options);
ins(1).system = 1;
ins(1).input = 1;
ins(2).system = 1;
ins(2).input = 2;
ins(3).system = 1;
ins(3).input = 3;
ins(4).system = 2;
ins(4).input = 1;
ins(5).system = 2;
ins(5).input = 3;
outs(1).system = 2;
outs(1).output = 1;
outs(2).system = 2;
outs(2).output = 2;
qp_sys = mimoCascade(pd,qp,[],ins,outs);
clear ins;

toffset = -1;
tt=-1;

torque_fade_in = 0.1; % sec, to avoid jumps at the start

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

% low pass filter for floating base velocities
alpha_v = 1;
float_v = 0;

l_foot_contact = 0;
r_foot_contact = 0;
qd_control = 0;
qd_filt = 0;
eta=1;

udes = zeros(nu,1);
qddes = zeros(nu,1);
qd_int_state = zeros(nq+4,1);
while tt<T
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    tau = x(2*nq+(1:nq));

    % low pass filter floating base velocities
    float_v = (1-alpha_v)*float_v + alpha_v*x(nq+(1:6));
    x(nq+(1:6)) = float_v;

    q = x(1:nq);
    qd = x(nq+(1:nq));
    %qt = fasteval(qtraj,tt);

    fc = output(fcb,tt,[],[q;qd]);
    if fc(1)~=l_foot_contact || fc(2)~=r_foot_contact
      % contact changed
      l_foot_contact = fc(1);
      r_foot_contact = fc(2);
      eta = 0;
    end
    qd_filt = 0.8*qd_filt + 0.2*qd;
    qd_control = (1-eta)*qd_filt + eta*qd;
    eta = min(1.0, eta+0.005);
    
    x_filt = [q;qd];
    
    junk = output(fshift,tt,[],x_filt);

    u_and_qdd = output(qp_sys,tt,[],[q0; x_filt; fc; x_filt; fc]);
    u=u_and_qdd(1:nu);
    qdd=u_and_qdd(nu+(1:nq));

    qd_int_state = mimoUpdate(vo,tt,qd_int_state,x_filt,qdd,fc);
    qd_ref = mimoOutput(vo,tt,qd_int_state,x_filt,qdd,fc);

    % fade in desired torques to avoid spikes at the start
    udes(joint_act_ind) = u(joint_act_ind);
    tau = tau(act_idx_map);
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);

    qddes(joint_act_ind) = qd_ref(joint_act_ind);

    ref_frame.publish(t,[q0(act_idx_map);qddes;udes],'ATLAS_COMMAND');
  end
end

disp('moving back to fixed point using position control.');
gains = getAtlasGains();
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,5);

end
