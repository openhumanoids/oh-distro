function atlasStepping
%NOTEST

joint_str = {'leg'};% <---- cell array of (sub)strings  

% inverse dynamics PD gains (only for input=position, control=force)
Kp = 20;
Kd = 5;

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
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
gains = getAtlasGains(input_frame); % change gains in this file

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

gains2 = getAtlasGains(input_frame); 
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains2.k_f_p(joint_act_ind); 
gains.ff_f_d(joint_act_ind) = gains2.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains2.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains2.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = gains.k_q_p(joint_act_ind)*0.4;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% get current state
[x,~] = getMessage(state_plus_effort_frame);
x0 = x(1:2*nq); 
q0 = x0(1:nq);
navgoal = [x0(1);x0(2);0;0;0;x0(6)];

% compute desired footstep and zmp trajectories
footstep_planner = FootstepPlanner(r);
step_options = footstep_planner.defaults;
step_options.max_num_steps = 2;
step_options.min_num_steps = 1;
step_options.step_speed = 0.01;
step_options.follow_spline = false;
step_options.right_foot_lead = true;
step_options.ignore_terrain = false;
step_options.nom_step_width = r.nom_step_width;
step_options.nom_forward_step = r.nom_forward_step;
step_options.max_forward_step = r.max_forward_step;
step_options.behavior = drc.walking_goal_t.BEHAVIOR_WALKING;
step_options.full_foot_pose_constraint = true;

footsteps = r.createInitialSteps(x0, navgoal, step_options);
for j = 1:length(footsteps)
  footsteps(j).pos = r.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
end
step_options.full_foot_pose_constraint = true;
[support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(r, x0, footsteps,step_options);
link_constraints = buildLinkConstraints(r, q0, foottraj);
 
[xtraj, ~, ~, ts] = robotWalkingPlan(r, q0, q0, zmptraj, comtraj, link_constraints);
T = ts(end);

qtraj = PPTrajectory(spline(ts,xtraj(1:nq,:)));

v = r.constructVisualizer;
for i=linspace(0,T,100)
  v.draw(i,qtraj.eval(i));
  pause(T/100/3);
end
qdtraj = fnder(qtraj,1);
qddtraj = fnder(qtraj,2);

% compute s1,s2 derivatives for controller Vdot computation
s1dot = fnder(V.s1,1);
s2dot = fnder(V.s2,1);

ctrl_data = SharedDataHandle(struct(...
  'A',[zeros(2),eye(2); zeros(2,4)],...
  'B',[zeros(2); eye(2)],...
  'C',[eye(2),zeros(2)],...
  'Qy',eye(2),...
  'R',zeros(2),...
  'is_time_varying',true,...
  'S',V.S.eval(0),... % always a constant
  's1',V.s1,...
  's2',V.s2,...
  's1dot',s1dot,...
  's2dot',s2dot,...
  'x0',[zmptraj.eval(T);0;0],...
  'u0',zeros(2,1),...
  'comtraj',comtraj,...
  'link_constraints',link_constraints, ...
  'support_times',support_times,...
  'supports',[supports{:}],...
  't_offset',0,...
  'mu',1,...
  'ignore_terrain',false,...
  'trans_drift',[0;0;0],...
  'y0',zmptraj));

% instantiate QP controller
options.dt = 0.003;
options.slack_limit = 30.0;
options.w = 0.001;
options.lcm_foot_contacts = false;
options.use_mex = true;
options.contact_threshold = 0.05;
qp = QPControlBlock(r,ctrl_data,options);

qddes = zeros(nu,1);
udes = zeros(nu,1);

toffset = -1;
tt=-1;
dt = 0.03;

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
    
    u = mimoOutput(qp,tt,[],qdddes,zeros(18,1),[q;qd]);
    udes(joint_act_ind) = u(joint_act_ind);
    
    % fade in desired torques to avoid spikes at the start
    alpha = min(1.0,tt/torque_fade_in);
    udes(joint_act_ind) = (1-alpha)*tau(joint_act_ind) + alpha*udes(joint_act_ind);
    
    % compute desired velocity
    qddes_state_frame = qdtraj_t + pd*dt;
    qddes_input_frame = qddes_state_frame(act_idx_map);
    qddes(joint_act_ind) = qddes_input_frame(joint_act_ind);
    
    ref_frame.publish(t,[qdes;qddes;udes],'ATLAS_COMMAND');
  end
end

disp('moving back to fixed point using position control.');
gains = getAtlasGains(input_frame); % change gains in this file
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_plus_effort_frame,ref_frame,act_idx_map,6);

end
