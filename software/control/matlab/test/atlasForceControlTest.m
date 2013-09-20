function atlasForceControlTest
%NOTEST

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint = 'r_arm_usy';% <---- 
signal = 'zoh';% <----  zoh, foh, chirp

% SIGNAL PARAMS %%%%%%%%%%%%%
if strcmp( signal, 'chirp' )
  zero_crossing = true;
  ts = linspace(0,25,400);% <----
  amp = 10;% <----  Nm
  freq = linspace(0.025,0.4,400);% <----  cycles per second
else
  ts = linspace(0,200,5);% <----
  vals = [0 0 0 0 0];% <----  Nm 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=ts(end);

% load robot model
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

options.floating = false;
r_fixed = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
fixed_joint_idx = find(strcmp(r_fixed.getStateFrame.coordinates,joint));

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nq = getNumDOF(r);
nu = getNumInputs(r);

joint_index_map = struct(); % maps joint names to indices
joint_offset_map = struct(); % maps joint names to nominal angle offsets
joint_sign_map = struct(); % maps joint names to signs in the direction of desired motion
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
  joint_offset_map.(state_frame.coordinates{i}) = 0;
  joint_sign_map.(state_frame.coordinates{i}) = 1;
end

% set nonzero offsets
joint_offset_map.l_arm_shx = 0;
joint_offset_map.l_arm_ely = 1.57;
joint_offset_map.r_arm_shx = 0;
joint_offset_map.r_arm_ely = 1.57;
joint_offset_map.l_arm_elx = 1.57;
joint_offset_map.r_arm_elx = -1.57;
joint_offset_map.l_arm_uwy = 1.57;
joint_offset_map.r_arm_uwy = 1.57;


% set negative joints
joint_sign_map.l_arm_ely = -1;
joint_sign_map.r_arm_usy = -1;
joint_sign_map.r_arm_shx = -1;
joint_sign_map.r_arm_ely = -1;
joint_sign_map.r_arm_elx = -1;
joint_sign_map.r_arm_mwx = -1;

if ~isfield(joint_index_map,joint)
  error ('unknown joint name');
end

gains = getAtlasGains(input_frame); 

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% setup desired pose based on joint being tuned
qdes = zeros(nq,1);

if strcmp(joint,'l_arm_shx') 
  
  qdes(joint_index_map.r_arm_shx) = 1.45;

  
elseif strcmp(joint,'r_arm_shx')
  
  qdes(joint_index_map.l_arm_shx) = -1.45;


elseif strcmp(joint,'l_arm_usy') || strcmp(joint,'r_arm_usy')

  qdes(joint_index_map.r_arm_shx) = 1.0;
  qdes(joint_index_map.l_arm_shx) = -1.0;

  qdes(joint_index_map.l_arm_uwy) = 1.57;
  qdes(joint_index_map.r_arm_uwy) = 1.57;

  qdes(joint_index_map.r_arm_ely) = joint_offset_map.r_arm_ely;
  qdes(joint_index_map.l_arm_ely) = joint_offset_map.l_arm_ely;

elseif strcmp(joint,'l_arm_ely') || strcmp(joint,'l_arm_mwx') || strcmp(joint,'l_arm_elx')
  
  qdes(joint_index_map.r_arm_shx) = 1.45;
  qdes(joint_index_map.l_arm_elx) = 1.57;
  qdes(joint_index_map.l_arm_ely) = 3.14;

elseif strcmp(joint,'r_arm_ely') || strcmp(joint,'r_arm_mwx') || strcmp(joint,'r_arm_elx')
  
  qdes(joint_index_map.l_arm_shx) = -1.45;
  qdes(joint_index_map.r_arm_elx) = -1.57;
  qdes(joint_index_map.r_arm_uwy) = 1.57;
  qdes(joint_index_map.r_arm_ely) = 3.14;

elseif strcmp(joint,'r_arm_uwy')
  
  qdes(joint_index_map.l_arm_shx) = -1.45;
  qdes(joint_index_map.r_arm_ely) = 1.57;
  qdes(joint_index_map.r_arm_elx) = -1.57;
  qdes(joint_index_map.r_arm_uwy) = 1.57;
  qdes(joint_index_map.r_arm_mwx) = 1.15;
  
elseif strcmp(joint,'l_arm_uwy')
  
  qdes(joint_index_map.r_arm_shx) = 1.45;
  qdes(joint_index_map.l_arm_ely) = 1.57;
  qdes(joint_index_map.l_arm_elx) = 1.57;
  qdes(joint_index_map.l_arm_uwy) = 1.57;
  qdes(joint_index_map.l_arm_mwx) = -1.15;
  
else
  error ('that joint isnt supported yet');
end

qdes(joint_index_map.(joint)) = joint_offset_map.(joint);

act_idx = getActuatedJoints(r);
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,4);

gains2 = getAtlasGains(input_frame); 
gains.k_f_p(act_idx==joint_index_map.(joint)) = gains2.k_f_p(act_idx==joint_index_map.(joint));
gains.ff_f_d(act_idx==joint_index_map.(joint)) = gains2.ff_f_d(act_idx==joint_index_map.(joint));
gains.ff_qd(act_idx==joint_index_map.(joint)) = gains2.ff_qd(act_idx==joint_index_map.(joint));
gains.ff_const(act_idx==joint_index_map.(joint)) = gains2.ff_const(act_idx==joint_index_map.(joint));
% set joint position gains to 0
gains.k_q_p(act_idx==joint_index_map.(joint)) = 0;
gains.k_q_i(act_idx==joint_index_map.(joint)) = 0;
gains.k_qd_p(act_idx==joint_index_map.(joint)) = 0;

ref_frame.updateGains(gains);
udes = zeros(nu,1);

if ~exist('vals','var')
  vals=amp;
end

vals = joint_sign_map.(joint) * vals;
if strcmp(signal,'zoh')
  input_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
  input_traj = PPTrajectory(foh(ts,vals));
elseif strcmp(signal,'chirp')
  offset = 0;
  if zero_crossing
  	input_traj = PPTrajectory(foh(ts, offset + amp*sin(ts.*freq*2*pi)));
  else
    input_traj = PPTrajectory(foh(ts, offset + joint_sign_map.(joint)*(0.5*amp - 0.5*amp*cos(ts.*freq*2*pi))));
  end
else
  error('unknown signal');
end

toffset = -1;
tt=-1;

H = [1 0];
R = 5e-4;

jest = zeros(2,1);
P = eye(2);

joint_state_ind = strcmp(joint,r.getStateFrame.coordinates);

fixed_act_idx = ~cellfun(@isempty,strfind(r_fixed.getInputFrame.coordinates,joint));


while tt<T
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      tlast=0;
    end
    tt=t-toffset;
    dt = tt-tlast;
    
    F = [1 dt; 0 1];
    Q = 0.3*diag([dt 1]);
    
    % compute filtered velocity
    jprior = F*jest;
    Pprior = F*P*F' + Q;
    meas_resid = x(joint_state_ind) - H*jprior;
    S = H*Pprior*H' + R;
    K = (P*H')*inv(S);
    jest = jprior + K*meas_resid;
    P = (eye(2) - K*H)*Pprior;
    
    
    if jest(2) > 0
      Fc = 11.0;
    else
      Fc = 11.0;
    end
    Fv = 0.5;
    Fc_window = 0.175;


    tau_friction = max(-1,min(1,jest(2)/Fc_window)) * Fc + Fv*jest(2); 

    % do inverse dynamics on fixed base model
    nq = getNumDOF(r_fixed);
    [Hd,C,B] = manipulatorDynamics(r_fixed,x(6+(1:nq)),x(nq+12+(1:nq)));
    qddot_des = zeros(nq,1);
    u = B\(Hd*qddot_des + C);
    f_grav = u(fixed_act_idx);
    
    % send torque command
    udes(act_idx==joint_index_map.(joint)) = input_traj.eval(tt) + tau_friction + f_grav;
    ref_frame.publish(t,[qdes(act_idx);udes],'ATLAS_COMMAND');
    tlast =tt;
  end
end
end
