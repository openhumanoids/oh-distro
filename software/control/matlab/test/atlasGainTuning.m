function atlasGainTuning
%NOTEST

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


% joint signs:
%
% l_usy   + (offset 0)
% l_shx   + (offset -1.45)
% l_elx   + (offset 0)
% l_ely   - (offset 1.57)
% l_uwy   + (offset 0)
% l_mwx   + (offset 0)

% l_usy   - (offset 0)
% r_shx   - (offset 1.45)
% r_ely   - (offset 1.57)
% r_elx   - (offset 0)
% r_uwy   + (offset 0)
% r_mwx   - (offset 0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint = 'l_arm_mwx';% <---- 
control_mode = 'position';% <----  force, position
signal = 'zoh';% <----  zoh, foh, chirp

% GAINS %%%%%%%%%%%%%%%%%%%%%
ff_const = -0.1;% <----
if strcmp(control_mode,'force')
  % force gains: only have an effect if control_mode==force
  k_f_p = 0.0;% <----
  ff_f_d = 0.0;% <----
  ff_qd = 0.0;% <----
elseif strcmp(control_mode,'position')  
  % position gains: only have an effect if control_mode==position
  k_q_p =  10.0;% <----
  k_q_i = 0.0;% <----
  k_qd_p = 0.75;% <----
else
  error('unknown control mode');
end

% SIGNAL PARAMS %%%%%%%%%%%%%
if strcmp( signal, 'chirp' )
  ts = linspace(0,25,400);% <----
  amp = 0.2;% <----  Nm or radians
  freq = linspace(0.025,0.4,400);% <----  cycles per second
else
  ts = linspace(0,15,5);% <----
  vals = [0 0.1 0.2 0.1 0];% <----  Nm or radians
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T=ts(end);

% check gain ranges --- TODO: make this more conservative
rangecheck(ff_const,-10,10);
if strcmp(control_mode,'force')
  rangecheck(k_f_p,0.5);
  rangecheck(ff_f_d,0,1);
  rangecheck(ff_qd,0,1);
elseif strcmp(control_mode,'position')  
  rangecheck(k_q_p,0,50);
  rangecheck(k_q_i,0,0.5);
  rangecheck(k_qd_p,0,50);
end

% check value ranges --- TODO: should be joint specific
if ~exist('vals','var')
  vals=amp;
end
if strcmp(control_mode,'force')
  rangecheck(vals,-70,70);
%   if any(~rangecheck(vals,-50,50))
%     disp('Warning: about to command relatively high torque. Ctrl+C to cancel.');
%     keyboard;
%   end
elseif strcmp(control_mode,'position')  
  rangecheck(vals,-pi,pi);
%   if any(~rangecheck(vals,-1,1))
%     disp('Warning: about to command relatively large position change. Ctrl+C to cancel.');
%     keyboard;
%   end
end

% load robot model
options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nu = getNumInputs(r);

atlas_joints = struct(); % maps joint names to indices
for i=1:nu
  atlas_joints.(input_frame.coordinates{i}) = i;
end
if ~isfield(atlas_joints,joint)
  error ('unknown joint name');
end

gains = getAtlasGains(input_frame);

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% setup desired pose based on joint being tuned
qdes = zeros(nu,1);
if strcmp(joint,'l_arm_shx') || strcmp(joint,'r_arm_shx') || ...
    strcmp(joint,'l_arm_elx') || strcmp(joint,'r_arm_elx') || ...
    strcmp(joint,'l_leg_hpy') || strcmp(joint,'r_arm_hpy') || ... 
    strcmp(joint,'l_leg_kny') || strcmp(joint,'r_arm_kny') 

  qdes(atlas_joints.r_arm_shx) = 1.45;
  qdes(atlas_joints.l_arm_shx) = -1.45;
  
elseif strcmp(joint,'l_arm_usy') || strcmp(joint,'r_arm_usy') || ...
    strcmp(joint,'l_arm_uwy') || strcmp(joint,'r_arm_uwy') || ...
    strcmp(joint,'l_arm_mwx') || strcmp(joint,'r_arm_mwx')
  
  qdes(atlas_joints.r_arm_shx) = 1.3;
  qdes(atlas_joints.l_arm_shx) = -1.3;
  
elseif strcmp(joint,'l_arm_ely')
  
  qdes(atlas_joints.r_arm_shx) = 1.45;
  qdes(atlas_joints.l_arm_elx) = 1.57;
%   qdes(atlas_joints.l_arm_ely) = 3.14;
  qdes(atlas_joints.l_arm_ely) = 1.57;
  
elseif strcmp(joint,'r_arm_ely')
  
  qdes(atlas_joints.l_arm_shx) = -1.45;
  qdes(atlas_joints.r_arm_elx) = -1.57;
%  qdes(atlas_joints.r_arm_ely) = 3.14;
  qdes(atlas_joints.r_arm_ely) = 1.57;

else
  error ('that joint isnt supported yet');
end

act_idx = getActuatedJoints(r);

% move to desired pos
movetime = 4.0;
toffset = -1;
tt=-1;
while tt<movetime
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      q0 = x(act_idx);
      qdes_traj = PPTrajectory(foh([0,movetime],[q0,qdes]));
    end
    tt=t-toffset;
    q_d = qdes_traj.eval(tt);
    ref_frame.publish(t,[q_d;0*q_d],'ATLAS_COMMAND');
  end
end

disp('Ready to send input signal.');
keyboard;

% set gains to user specified values
gains.ff_const(atlas_joints.(joint)) = ff_const;
if strcmp(control_mode,'force')
  % set force gains
  gains.k_f_p(atlas_joints.(joint)) = k_f_p; 
  gains.ff_f_d(atlas_joints.(joint)) = ff_f_d;
  gains.ff_qd(atlas_joints.(joint)) = ff_qd;
  % set joint position gains to 0
  gains.k_q_p(atlas_joints.(joint)) = 0;
  gains.k_q_i(atlas_joints.(joint)) = 0;
  gains.k_qd_p(atlas_joints.(joint)) = 0;
elseif strcmp(control_mode,'position')  
  % set force gains to 0
  gains.k_f_p(atlas_joints.(joint)) = 0; 
  gains.ff_f_d(atlas_joints.(joint)) = 0;
  gains.ff_qd(atlas_joints.(joint)) = 0;
  % set joint position gains 
  gains.k_q_p(atlas_joints.(joint)) = k_q_p;
  gains.k_q_i(atlas_joints.(joint)) = k_q_i;
  gains.k_qd_p(atlas_joints.(joint)) = k_qd_p;
else
  error('unknown control mode');
end 

ref_frame.updateGains(gains);
udes = zeros(nu,1);

if strcmp(signal,'zoh')
  input_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
  input_traj = PPTrajectory(foh(ts,vals));
elseif strcmp(signal,'chirp')
%   input_traj = PPTrajectory(foh(ts,amp*sin(ts.*freq*2*pi)));
  input_traj = PPTrajectory(foh(ts, 0.5*amp + 0.5*amp*sin(ts.*freq*2*pi)));
else
  error('unknown signal');
end

toffset = -1;
tt=-1;
while tt<T
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    if strcmp(control_mode,'force')
      udes(atlas_joints.(joint)) = input_traj.eval(tt);
    elseif strcmp(control_mode,'position')
      qdes(atlas_joints.(joint)) = input_traj.eval(tt);
    end
    ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
  end
end

end
