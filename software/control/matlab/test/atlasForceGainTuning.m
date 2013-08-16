function atlasForceGainTuning
%NOTEST

% for tuning joint force gains

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nu = getNumInputs(r);

atlas_joints = struct();   % maps joint names to indices
for i=1:nu
  atlas_joints.(input_frame.coordinates{i}) = i;
end

gains = getAtlasGains(input_frame);

% zero out force gains to start --- tuning one joint at a time
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint = 'l_leg_hpy'; 

k_f_p = 0.05; 
ff_f_d = 0.0;
ff_qd = 0.0;

signal = 'foh'; % zoh, foh, chirp
ts = linspace(0,30,5);
vals = [0 0 5 0 0];
T = ts(end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if ~isfield(atlas_joints,joint)
  error ('unknown joint name');
end

% setup arm pose
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
  
  qdes(atlas_joints.r_arm_shx) = 1.35;
  qdes(atlas_joints.l_arm_shx) = -1.35;

elseif strcmp(joint,'l_arm_ely')
  
  qdes(atlas_joints.r_arm_shx) = 1.45;
  qdes(atlas_joints.l_arm_elx) = 1.57;
  qdes(atlas_joints.l_arm_ely) = 3.14;

elseif strcmp(joint,'r_arm_ely')
  
  qdes(atlas_joints.l_arm_shx) = -1.45;
  qdes(atlas_joints.r_arm_elx) = -1.57;
  qdes(atlas_joints.r_arm_ely) = 3.14;

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

% set joint position gain to 0
gains.k_q_p(atlas_joints.(joint)) = 0;

% set gains
gains.k_f_p(atlas_joints.(joint)) = k_f_p; 
gains.ff_f_d(atlas_joints.(joint)) = ff_f_d;
gains.ff_qd(atlas_joints.(joint)) = ff_qd;

ref_frame.updateGains(gains);

udes=zeros(nu,1);
if strcmp(signal,'zoh')
  udes_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
  udes_traj = PPTrajectory(foh(ts,vals));
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
    udes(atlas_joints.(joint)) = udes_traj.eval(tt);
    ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
  end
end

end
