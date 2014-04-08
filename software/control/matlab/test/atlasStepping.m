function atlasStepping
%NOTEST
addpath(fullfile(getDrakePath,'examples','ZMP'));

joint_str = {'leg'};% <---- cell array of (sub)strings  

% inverse dynamics PD gains (only for input=position, control=force)
Kp = 30;
Kd = 2;

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_bdi_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
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

gains2 = getAtlasGains(); 
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains2.k_f_p(joint_act_ind); 
gains.ff_f_d(joint_act_ind) = gains2.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains2.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains2.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = gains.k_q_p(joint_act_ind)*0.2;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq); 

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
request.params.max_num_steps = 3;
request.params.min_num_steps = 1;
request.params.min_step_width = 0.2;
request.params.nom_step_width = 0.26;
request.params.max_step_width = 0.39;
request.params.nom_forward_step = 0.2;
request.params.max_forward_step = 0.45;
request.params.ignore_terrain = false;
request.params.planning_mode = request.params.MODE_AUTO;
request.params.behavior = request.params.BEHAVIOR_WALKING;
request.params.map_command = 0;
request.params.leading_foot = request.params.LEAD_RIGHT;
request.default_step_params = drc.footstep_params_t();
request.default_step_params.step_speed = 0.025;
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
walking_ctrl_data.supports = walking_ctrl_data.supports{1}; % TODO: fix this

% ts = 0:0.1:walking_ctrl_data.zmptraj.tspan(end);
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

ctrl_data = SharedDataHandle(struct(...
  'A',[zeros(2),eye(2); zeros(2,4)],...
  'B',[zeros(2); eye(2)],...
  'C',[eye(2),zeros(2)],...
  'Qy',eye(2),...
  'R',zeros(2),...
  'is_time_varying',true,...
  'S',walking_ctrl_data.S.eval(0),... % always a constant
  's1',walking_ctrl_data.s1,...
  's2',walking_ctrl_data.s2,...
  's1dot',walking_ctrl_data.s1dot,...
  's2dot',walking_ctrl_data.s2dot,...
  'x0',[walking_ctrl_data.zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',walking_ctrl_data.comtraj,...
  'link_constraints',walking_ctrl_data.link_constraints, ...
  'support_times',walking_ctrl_data.support_times,...
  'supports',[walking_ctrl_data.supports{:}],...
  't_offset',0,...
  'mu',walking_ctrl_data.mu,...
  'ignore_terrain',walking_ctrl_data.ignore_terrain,...
  'qp_active_set',0,...
  'trans_drift',[0;0;0],...
  'y0',walking_ctrl_data.zmptraj));

traj = PPTrajectory(spline(ts,walking_plan.xtraj));
traj = traj.setOutputFrame(r.getStateFrame);

v = r.constructVisualizer;
playback(v,traj,struct('slider',true));

qtraj = PPTrajectory(foh(ts,walking_plan.xtraj(1:nq,:)));
qdtraj = fnder(qtraj,1);
qddtraj = fnder(qtraj,2);

% instantiate QP controller
options.dt = 0.003;
options.slack_limit = 30.0;
options.w = 0.005;
options.lcm_foot_contacts = false;
options.use_mex = true;
options.contact_threshold = 0.05;
options.output_qdd = true;
qp = QPControlBlock(r,ctrl_data,options);

qddes = zeros(nu,1);
udes = zeros(nu,1);

toffset = -1;
tt=-1;
dt = 0.001;

process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

torque_fade_in = 0.75; % sec, to avoid jumps at the start

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

while tt<T+2
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;

    tau = x(2*nq+(1:nq));
    tau = tau(act_idx_map);
    
    % get estimated state
    kf_state = kf.update(tt,kf_state,x(1:nq));
    x = kf.output(tt,kf_state,x(1:nq));

    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    % get desired configuration
    qt = qtraj.eval(tt);
    qdes = qt(act_idx_map);

    qt(6) = q(6); % ignore yaw
    
    % get desired acceleration, open loop + PD
    qdtraj_t = qdtraj.eval(tt);
    pd = Kp*(qt-q) + Kd*(qdtraj_t-qd);
    qdddes = qddtraj.eval(tt) + pd;
    
    [u,qdd] = mimoOutput(qp,tt,[],qdddes,zeros(18,1),[q;qd]);
    udes(joint_act_ind) = u(joint_act_ind);
    
    % fade in desired torques to avoid spikes at the start
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);
    
    % compute desired velocity
    qddes_state_frame = qdtraj_t + qdd*dt;
    qddes_input_frame = qddes_state_frame(act_idx_map);
    qddes(joint_act_ind) = qddes_input_frame(joint_act_ind);
    
    ref_frame.publish(t,[qdes;qddes;udes],'ATLAS_COMMAND');
  end
end

disp('moving back to fixed point using position control.');
gains = getAtlasGains(); % change gains in this file
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,6);

end
