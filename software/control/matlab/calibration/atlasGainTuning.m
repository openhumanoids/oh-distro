function atlasGainTuning
%NOTEST

% simple function for tuning position and torque control gains joint-by-joint

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET JOINT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
joint = 'back_bkx';% <---- joint name 
input_mode = 'position';% <---- force, position
control_mode = 'force+velocity';% <---- force, force+velocity, position
signal = 'chirp';% <----  zoh, foh, chirp

% INPUT SIGNAL PARAMS %%%%%%%%%%%%%
T = 25;% <--- signal duration (sec)

% chirp specific
amp = 0.15;% <----  Nm or radians
chirp_f0 = 0.05;% <--- chirp starting frequency
chirp_fT = 0.35;% <--- chirp ending frequency
chirp_sign = 0;% <--- -1: below offset, 1: above offset, 0: centered about offset 

% z/foh
vals = 0.2*[0 1 1 -1 -1 0 0];% <----  Nm or radians

% inverse dynamics PD gains (only for input=position, control=force)
Kp = 100;
Kd = 8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if strcmp(signal, 'chirp')
  ts = linspace(0,T,800);
else
  ts = linspace(0,T,length(vals));
end

% load robot model
options.floating = true;
options.ignore_friction = false;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% load fixed-base model
options.floating = false;
r_fixed = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));

% setup frames
state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasPosVelTorqueRef(r);

nq = getNumDOF(r);
nu = getNumInputs(r);
nq_fixed = getNumDOF(r_fixed);

joint_index_map = cell2struct(num2cell(1:nq),state_frame.coordinates(1:nq),2);
joint_index_map_fixed = cell2struct(num2cell(1:nq),r_fixed.getStateFrame.coordinates(1:nq),2);

act_idx = getActuatedJoints(r);
act_idx_fixed = getActuatedJoints(r_fixed);

% check value ranges --- TODO: should be joint specific
if strcmp(signal,'chirp')
  vals=amp;
end
if strcmp(input_mode,'force')
%   rangecheck(vals,-200,200);
  if ~rangecheck(vals,-50,50)
    resp = input('Warning: about to command relatively high torque. OK? (y/n): ','s');
    if ~strcmp(resp,{'y','yes'})
      return;
    end
  end
elseif strcmp(input_mode,'position')  
  rangecheck(vals,-3.2,3.2);
  if ~rangecheck(vals,-1,1)
    resp = input('Warning: about to command relatively large position change. OK? (y/n): ','s');
    if ~strcmp(resp,{'y','yes'})
      return;
    end
  end
end

if ~isfield(joint_index_map,joint)
  error ('unknown joint name');
end

gains = getAtlasGains();

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
gains.ff_qd_d = zeros(nu,1);
ref_frame.updateGains(gains);

% setup desired pose based on joint being tuned
[qdes,motion_sign] = getAtlasJointMotionConfig(r,joint,2);

% move to desired pos
atlasLinearMoveToPos(qdes,state_frame,ref_frame,act_idx,3);

gains2 = getAtlasGains();
if any(strcmp(control_mode,{'force','force+velocity'}))
  % set joint position gains to 0
  gains.k_q_p(act_idx==joint_index_map.(joint)) = 0;
  gains.k_q_i(act_idx==joint_index_map.(joint)) = 0;
  gains.k_qd_p(act_idx==joint_index_map.(joint)) = 0;
  % set force gains
  gains.k_f_p(act_idx==joint_index_map.(joint)) = gains2.k_f_p(act_idx==joint_index_map.(joint));
  gains.ff_f_d(act_idx==joint_index_map.(joint)) = gains2.ff_f_d(act_idx==joint_index_map.(joint));
  gains.ff_qd(act_idx==joint_index_map.(joint)) = gains2.ff_qd(act_idx==joint_index_map.(joint));
  gains.ff_qd_d(act_idx==joint_index_map.(joint)) = gains2.ff_qd_d(act_idx==joint_index_map.(joint));
  ref_frame.updateGains(gains);
end
 
vals = motion_sign * vals;
if strcmp(input_mode,'position')
  offset = qdes(joint_index_map.(joint));
  vals = offset + vals;  
else
  offset = 0;
end

if strcmp(signal,'zoh')
  input_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
  input_traj = PPTrajectory(foh(ts,vals));
elseif strcmp(signal,'chirp')
  input_traj = chirpTraj(amp,chirp_f0,chirp_fT,T,0,chirp_sign*motion_sign);
  fade_window = 2; % sec
  fader = PPTrajectory(foh([0 fade_window T-fade_window T],[0 1 1 0]));
  input_traj = offset + fader*input_traj;
else
  error('unknown signal');
end

inputd_traj = fnder(input_traj,1);
inputdd_traj = fnder(input_traj,2);

qdes=qdes(act_idx); % convert to input frame
qddes=zeros(nu,1); 
udes = zeros(nu,1);
toffset = -1;
tt=-1;
dt = 0.0025;

qd_int = 0;
tt_prev=-1;
while tt<T
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    tt=t-toffset;
    if tt_prev==-1
      dt = 0.0025;
    else
      dt = 0.99*dt + 0.01*(tt-tt_prev);
    end
    tt_prev=tt;
    if strcmp(input_mode,'force')
      udes(act_idx==joint_index_map.(joint)) = input_traj.eval(tt);
    elseif strcmp(input_mode,'position')
      jdes = input_traj.eval(tt);
      qdes(act_idx==joint_index_map.(joint)) = jdes;
      
      if any(strcmp(control_mode,{'force','force+velocity'}))
        % do inverse dynamics on fixed base model
        
        q = x(6+(1:nq_fixed));
        qd = x(nq_fixed+12+(1:nq_fixed));
        [H,C,B] = manipulatorDynamics(r_fixed,q,qd);

        jddes = inputd_traj.eval(tt)*0;
        jdddes = inputdd_traj.eval(tt)*0;

        qddot_des = zeros(nq_fixed,1);
        fidx = joint_index_map_fixed.(joint);
        qddot_des(fidx) = jdddes + Kp*(jdes-q(fidx)) + Kd*(jddes-qd(fidx));

        u = B\(H*qddot_des + C);
        udes(act_idx==joint_index_map.(joint)) = u(act_idx_fixed==joint_index_map_fixed.(joint));

        qd_int = qd_int + qddot_des(fidx)*dt;

        if strcmp(control_mode,'force+velocity')
          qddes(act_idx==joint_index_map.(joint)) = qd_int-qd(fidx); %jddes + Kp*(jdes-q(fidx))*dt + Kd*(jddes-qd(fidx))*dt;
        end
      end
        
    end
    ref_frame.publish(t,[qdes;qddes;udes],'ATLAS_COMMAND');
  end
end
ref_frame.publish(t,[qdes;0*qddes;udes],'ATLAS_COMMAND');

end