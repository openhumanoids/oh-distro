function atlasStandingLegTuning
%NOTEST

% simple function for tuning position and torque control gains
% while standing using position and force control for different subsets of
% joints

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT/MOVEMENT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint_str = {'l_leg_kny'};% <---- cell array of (sub)strings  

% INPUT SIGNAL PARAMS %%%%%%%%%%%%%
dim = 3; % what spatial dimension to move COM: x/y/z (1/2/3)
T = 10;% <--- signal duration (sec)

% chirp params
amp = 0.0;% <---- meters, COM DELTA
chirp_f0 = 0.05;% <--- chirp starting frequency
chirp_fT = 0.1;% <--- chirp ending frequency
chirp_sign = -1;% <--- -1: negative, 1: positive, 0: centered about offset 

% inverse dynamics PD gains (only for input=position, control=force)
Kp = 25;
Kd = 4;
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
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

act_idx_map = getActuatedJoints(r);
gains = getAtlasGains(input_frame); % change gains in this file

joint_ind = [];
joint_act_ind = [];
for i=1:length(joint_str)
  joint_ind = union(joint_ind,find(~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),joint_str{i}))));
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
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx_map,5);

gains2 = getAtlasGains(input_frame); 
% reset force gains for joint being tuned
gains.k_f_p(joint_act_ind) = gains2.k_f_p(joint_act_ind); 
gains.ff_f_d(joint_act_ind) = gains2.ff_f_d(joint_act_ind);
gains.ff_qd(joint_act_ind) = gains2.ff_qd(joint_act_ind);
gains.ff_qd_d(joint_act_ind) = gains2.ff_qd_d(joint_act_ind);
% set joint position gains to 0 for joint being tuned
gains.k_q_p(joint_act_ind) = 0;
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
rfoot_body = r.findLinkInd('r_foot');
lfoot_body = r.findLinkInd('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0]);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0]);

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

v = r.constructVisualizer;
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    kc_com = constructPtrWorldCoMConstraintmex(r.getMexModelPtr,comtraj.eval(t),comtraj.eval(t));
    rfarg = {constructPtrWorldPositionConstraintmex(r.getMexModelPtr,rfoot_body,[0;0;0],rfoot0,rfoot0),...
      constructPtrWorldEulerConstraintmex(r.getMexModelPtr,rfoot_body,[0;0;0],[0;0;0])};
    lfarg = {constructPtrWorldPositionConstraintmex(r.getMexModelPtr,lfoot_body,[0;0;0],lfoot0,lfoot0),...
      constructPtrWorldEulerConstraintmex(r.getMexModelPtr,lfoot_body,[0;0;0],[0;0;0])};
    q(:,i) = inverseKin(r,q(:,i-1),q0,kc_com,rfarg{:},lfarg{:},ikoptions);
  else
    q = q0;
  end
  v.draw(t,q(:,i));
end
qtraj = PPTrajectory(spline(ts,q));
qdtraj = fnder(qtraj,1);
qddtraj = fnder(qtraj,2);

resp = input('OK to send input to robot? (y/n): ','s');
if ~strcmp(resp,{'y','yes'})
  return;
end

% set up QP controller params---exclude ZMP for now
foot_support = SupportState(r,find(~cellfun(@isempty,strfind(r.getLinkNames(),'foot'))));
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

qddes_prev = zeros(nu,1);
udes_prev = zeros(nu,1);

toffset = -1;
tt=-1;
dt = 0.03;

process_noise = 0.3;
observation_noise = 5e-4;
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

while tt<T+2
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      xy_offset = x(1:2); % because state estimate will not be 0,0 to start
    end
    tt=t-toffset;
    
    % get estimated state
	kf_state = kf.update(tt,kf_state,x(1:nq));
	x = kf.output(tt,kf_state,x(1:nq))

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

    % compute desired velocity
    qddes_state_frame = qdtraj_t + pd*dt;
    qddes_input_frame = qddes_state_frame(act_idx_map);
    qddes(joint_act_ind) = qddes_input_frame(joint_act_ind);
    
    % low pass filter inputs
    alpha = 0.1;
    udes = (1-alpha)*udes_prev + alpha*udes; 
    qddes = (1-alpha)*qddes_prev + alpha*qddes; 
    
    ref_frame.publish(t,[qdes;qddes;udes],'ATLAS_COMMAND');

    udes_prev = udes;
    qddes_prev = qddes;
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
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx_map,4);

end
