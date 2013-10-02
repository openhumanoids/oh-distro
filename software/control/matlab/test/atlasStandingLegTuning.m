function atlasStandingLegTuning
%NOTEST

% simple function for tuning position and torque control gains
% joint-by-joint while standing using position control

% gain spec: 
% q, qd, f are sensed position, velocity, torque, from AtlasJointState
%
% q_d, qd_d, f_d are desired position, velocity, torque, from
% AtlasJointDesired
%
% The final joint command will be:
%
%  k_q_p   * ( q_d - q ) +
%  k_q_i   * 1/s * ( q_d - q ) +
%  k_qd_p  * ( qd_d - qd ) +
%  k_f_p   * ( f_d - f ) +
%  ff_qd   * qd +
%  ff_qd_d * qd_d +
%  ff_f_d  * f_d +
%  ff_const


% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = r.setInitialState(xstar);
v = r.constructVisualizer;
% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
end

act_idx = getActuatedJoints(r);
gains = getAtlasGains(input_frame); % change gains in this file

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint = 'r_leg_hpz';% <---- 
control_mode = 'force';% <----  force, position
signal = 'chirp';% <----  zoh, foh, chirp

% SIGNAL PARAMS %%%%%%%%%%%%%
dim = 3; % what spatial dimension to move COM: x/y/z (1/2/3)
if strcmp( signal, 'chirp' )
  zero_crossing = false;
  ts = linspace(0,30,500);% <----
  amp = -0.15;% <---- meters, COM DELTA
  freq = linspace(0.025,0.1,500);% <----  cycles per second
else
  vals = 0.02*[0 0 1 0 0];% <---- meters, COM DELTA
  ts = linspace(0,30,length(vals));% <----
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=ts(end);

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% move to fixed point configuration 
qdes = xstar(1:nq);
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,3);

gains2 = getAtlasGains(input_frame); 
% reset force gains for joint being tuned
gains.k_f_p(act_idx==joint_index_map.(joint)) = gains2.k_f_p(act_idx==joint_index_map.(joint)); 
gains.ff_f_d(act_idx==joint_index_map.(joint)) = gains2.ff_f_d(act_idx==joint_index_map.(joint));
gains.ff_qd(act_idx==joint_index_map.(joint)) = gains2.ff_qd(act_idx==joint_index_map.(joint));
% set joint position gains to 0 for joint being tuned
gains.k_q_p(act_idx==joint_index_map.(joint)) = 0;
gains.k_q_i(act_idx==joint_index_map.(joint)) = 0;
gains.k_qd_p(act_idx==joint_index_map.(joint)) = 0;

ref_frame.updateGains(gains);

% compute desired COM trajectory
x0 = r.getInitialState(); 
q0 = x0(1:nq);
com0 = getCOM(r,q0);
comtraj = ConstantTrajectory(com0);

if strcmp(signal,'zoh')
  input_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
  input_traj = PPTrajectory(foh(ts,vals));
elseif strcmp(signal,'chirp')
  offset = 0;
  if strcmp(control_mode,'position')
    offset=joint_offset_map.(joint);
  end
  if zero_crossing
  	input_traj = PPTrajectory(foh(ts, offset + amp*sin(ts.*freq*2*pi)));
  else
    input_traj = PPTrajectory(foh(ts, offset + 0.5*amp - 0.5*amp*cos(ts.*freq*2*pi)));
  end
else
  error('unknown signal');
end

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
qddtraj = fnder(qtraj,2);
keyboard;

% kalman filter params
H = [eye(nq) zeros(nq)];
R = 5e-4*eye(nq);
P = eye(2*nq);
x_est = zeros(2*nq,1);

udes = zeros(nu,1);
toffset = -1;
tt=-1;
while tt<T
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      tlast=0;
    end
    tt=t-toffset;
    dt = tt-tlast;
    tlast =tt;

    F = [eye(nq) dt*eye(nq); zeros(nq) eye(nq)];
    Q = 0.3*[dt*eye(nq) zeros(nq); zeros(nq) eye(nq)];
    
    % compute filtered velocity
    jprior = F*x_est;
    Pprior = F*P*F' + Q;
    meas_resid = x(1:nq) - H*jprior;
    S = H*Pprior*H' + R;
    K = (P*H')/S;
    x_est = jprior + K*meas_resid;
    P = (eye(2*nq) - K*H)*Pprior;
    
    q = x_est(1:nq);
    qd = x_est(nq+(1:nq));
    
    qt = qtraj.eval(tt);
    qdes = qt(act_idx);
    
    qdddes = qddtraj.eval(tt) + 20.0*(qt-q);
    [Hd,C,B] = manipulatorDynamics(r,q,qd);

    % .... need to solve a QP to get udes+contact forces s.t. feet do not
    % move
    
    ref_frame.publish(t,[qdes;0*udes],'ATLAS_COMMAND');
  end
end

end
