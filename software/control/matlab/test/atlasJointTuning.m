function atlasJointTuning
%NOTEST

% for tuning arm joint force gains

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosTorqueRef(r);

nu = getNumInputs(r);

l_arm_usy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_usy')));
l_arm_shx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_shx')));
l_arm_ely = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_ely')));
l_arm_elx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_elx')));
l_arm_uwy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_uwy')));
l_arm_mwx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_mwx')));

r_arm_usy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_usy')));
r_arm_shx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_shx')));
r_arm_ely = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_ely')));
r_arm_elx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_elx')));
r_arm_uwy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_uwy')));
r_arm_mwx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_mwx')));


gains = getAtlasGains(input_frame);

% zero out force gains to start --- tuning one joint at a time
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

% SET JOINT %%%%%%%%%%%%%%%%%%%%%
joint = l_arm_elx; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% setup arm pose
if joint==l_arm_shx || joint==r_arm_shx || joint==l_arm_elx || joint==r_arm_elx
  qdes = zeros(nu,1);
  qdes(r_arm_shx) = 1.5;
  qdes(l_arm_shx) = -1.5;
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
gains.k_q_p(joint) = 0;

% SET GAINS %%%%%%%%%%%%%%%%%%%%%
gains.k_f_p(joint) = 0.155; % elx
% gains.k_f_p(joint) = 0.105; % shx
gains.ff_f_d(joint) = 0.0;
gains.ff_qd(joint) = 0.0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ref_frame.updateGains(gains);


% MOVEMENT %%%%%%%%%%%%%%%%%%%%%
movement = 'foh';
%movement = 'zoh';
torque_range = [0 35]; % elx
% torque_range = [0 60]; % shx
T = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

udes=zeros(nu,1);
if strcmp(movement,'zoh')
  udes_traj = PPTrajectory(zoh(linspace(0,T,4),[torque_range(1) ...
    torque_range(2) torque_range(1) torque_range(1)]));
elseif strcmp(movement,'foh')
  udes_traj = PPTrajectory(foh(linspace(0,T,4),[torque_range(1) ...
    torque_range(2) torque_range(1) torque_range(1)]));
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
    udes(joint) = udes_traj.eval(tt);
    ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
  end
end

end
