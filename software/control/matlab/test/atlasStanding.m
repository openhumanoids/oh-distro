function atlasStanding
%NOTEST

% test function for standing atlas using the QP controller. using the
% joint_str variable, the user can select a subset of leg joints with which
% to do torque control.
% useful for tuning gains, testing basic balancing capabilities, or making
% atlas dance.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT/MOVEMENT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint_str = {'leg'};% <---- cell array of (sub)strings  

% INPUT SIGNAL PARAMS %%%%%%%%%%%%%
dim = 3; % what spatial dimension to move COM: x/y/z (1/2/3)
T = 10;% <--- signal duration (sec)

% chirp params
amp = 0.08;% <---- meters, COM DELTA
chirp_f0 = 1.0;% <--- chirp starting frequency
chirp_fT = 1.0;% <--- chirp ending frequency
chirp_sign = -1;% <--- -1: negative, 1: positive, 0: centered about offset 

% inverse dynamics PD gains (only for input=position, control=force)
Kp = 20;
Kd = 5;

% turn on/off ZMP objective
use_zmp = true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts = linspace(0,T,800);

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
gains.k_q_p(joint_act_ind) = gains.k_q_p(joint_act_ind)*0.3;
gains.k_q_i(joint_act_ind) = 0;
gains.k_qd_p(joint_act_ind) = 0;

ref_frame.updateGains(gains);

% compute desired COM trajectory
x0 = r.getInitialState(); 
q0 = x0(1:nq);
com0 = getCOM(r,q0);
comtraj = ConstantTrajectory(com0);

input_traj = chirpTraj(amp,chirp_f0,chirp_fT,T,0,chirp_sign);
fade_window = 2; % sec
fader = PPTrajectory(foh([0 fade_window T-fade_window T],[0 1 1 0]));
input_traj = fader*input_traj;

if dim==1
  comtraj = comtraj + [input_traj;0;0];
elseif dim==2
  comtraj = comtraj + [0;input_traj;0];
else
  comtraj = comtraj + [0;0;input_traj];
end

% get foot positions
kinsol = doKinematics(r,q0);
rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_ind,[0;0;0]);
lfoot0 = forwardKin(r,kinsol,lfoot_ind,[0;0;0]);

cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 1000;
cost.back_bkz = 10;
cost.back_bky = 100;
cost.back_bkx = 100;
cost.r_arm_usy = 10;
cost.r_arm_shx = 10;
cost.r_arm_ely = 10;
cost.r_arm_elx = 10;
cost.r_arm_uwy = 10;
cost.r_arm_mwx = 10;
cost.l_arm_usy = 10;
cost.l_arm_shx = 10;
cost.l_arm_ely = 10;
cost.l_arm_elx = 10;
cost.l_arm_uwy = 10;
cost.l_arm_mwx = 10;

cost = double(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(diag(cost(1:nq)));

for i=1:length(ts)
  t = ts(i);
  if (i>1)
    kc_com = constructPtrWorldCoMConstraintmex(r.getMexModelPtr,comtraj.eval(t),comtraj.eval(t));
    rfarg = {constructPtrWorldPositionConstraintmex(r.getMexModelPtr,rfoot_ind,[0;0;0],rfoot0,rfoot0),...
      constructPtrWorldEulerConstraintmex(r.getMexModelPtr,rfoot_ind,[0;0;0],[0;0;0])};
    lfarg = {constructPtrWorldPositionConstraintmex(r.getMexModelPtr,lfoot_ind,[0;0;0],lfoot0,lfoot0),...
      constructPtrWorldEulerConstraintmex(r.getMexModelPtr,lfoot_ind,[0;0;0],[0;0;0])};
    q(:,i) = inverseKin(r,q(:,i-1),q0,kc_com,rfarg{:},lfarg{:},ikoptions);
  else
    q = q0;
  end
end

% visualize trajectory
qtraj = PPTrajectory(spline(ts,q));

traj = [qtraj;0*qtraj];
traj = traj.setOutputFrame(r.getStateFrame);

v = r.constructVisualizer;
playback(v,traj,struct('slider',true));

qdtraj = fnder(qtraj,1);
qddtraj = fnder(qtraj,2);

% set up QP controller params
foot_support = SupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));

if use_zmp
  % build TI-ZMP controller
  q0 = x0(1:nq);

  foot_pos = contactPositions(r,q0, [rfoot_ind, lfoot_ind]);
  ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
  comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';%mean(foot_pos(1:2,ch(1:end-1)),2);
  
  limp = LinearInvertedPendulum(com0(3));
  [~,V] = lqr(limp,comgoal);

  ctrl_data = SharedDataHandle(struct(...
    'A',[zeros(2),eye(2); zeros(2,4)],...
    'B',[zeros(2); eye(2)],...
    'C',[eye(2),zeros(2)],...
    'D',-com0(3)/9.81*eye(2),...
    'Qy',eye(2),...
    'R',zeros(2),...
    'S',V.S,...
    's1',zeros(4,1),...
    's2',0,...
    'x0',[comgoal;0;0],...
    'u0',zeros(2,1),...
    'y0',comgoal,...
    'qtraj',q0,...
    'mu',1,...
    'ignore_terrain',false,...
    'is_time_varying',false,...
    'trans_drift',[0;0;0],...
    'support_times',0,...
    't_offset',0,...
    'supports',foot_support));
else
  ctrl_data = SharedDataHandle(struct(...
    'A',[zeros(2),eye(2); zeros(2,4)],...
    'B',[zeros(2); eye(2)],...
    'C',[eye(2),zeros(2)],...
    'D',-com0(3)/9.81*eye(2),...
    'Qy',zeros(2),...
    'R',zeros(2),...
    'S',zeros(4),...
    's1',zeros(4,1),...
    's2',0,...
    'x0',[com0(1:2);0;0],...
    'u0',zeros(2,1),...
    'y0',com0(1:2),...
    'qtraj',q0,...
    'mu',1,...
    'ignore_terrain',false,...
    'is_time_varying',false,...
    'trans_drift',[0;0;0],...
    'support_times',0,...
    'supports',foot_support));
end

% instantiate QP controller
options.slack_limit = 30.0;
options.w = 0.001;
options.lcm_foot_contacts = false;
options.use_mex = true;
options.contact_threshold = 0.05;
qp = QPControlBlock(r,ctrl_data,options);

xy_offset = [0;0];
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
      xy_offset = x(1:2); % because state estimate will not be 0,0 to start
    end
    tt=t-toffset;

    tau = x(2*nq+(1:nq));
    tau = tau(act_idx_map);
    
    % get estimated state
    kf_state = kf.update(tt,kf_state,x(1:nq));
    x = kf.output(tt,kf_state,x(1:nq));

    q = x(1:nq);
    q(1:2) = q(1:2)-xy_offset;
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
