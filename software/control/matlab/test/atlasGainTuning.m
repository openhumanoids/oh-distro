function atlasGainTuning
%NOTEST

% simple function for tuning position and torque control gains
% joint-by-joint

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
joint = 'l_arm_elx';% <----
control_mode = AtlasJointControlMode.FORCE;
signal = 'chirp';% <----  zoh, foh, chirp

% GAINS %%%%%%%%%%%%%%%%%%%%%
gains_for_joint_to_tune = getGainsForJointToTune(control_mode);

% SIGNAL PARAMS %%%%%%%%%%%%%
if strcmp( signal, 'chirp' )
    zero_crossing = false;
    ts = linspace(0,25,400);% <----
    amp = 25;% <----  Nm or radians
    freq = linspace(0.05,0.3,400);% <----  cycles per second
else
    ts = linspace(0,20,7);% <----
    vals = [0 0 20 0 -20 0 0];% <----  Nm or radians
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

checkGainRanges(gains_for_joint_to_tune, control_mode);


% check value ranges --- TODO: should be joint specific
if ~exist('vals','var')
    vals=amp;
end

checkReferenceRanges(vals, control_mode);

% load robot model
options.floating = true;
robot = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

% setup frames
state_frame = getStateFrame(robot);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(robot);
ref_frame = AtlasPosTorqueRef(robot);

nu = getNumInputs(robot);
nq = getNumDOF(robot);

joint_index_map = struct(); % maps joint names to indices
joint_offset_map = struct(); % maps joint names to nominal angle offsets
for i=1:nq
    joint_index_map.(state_frame.coordinates{i}) = i;
    joint_offset_map.(state_frame.coordinates{i}) = 0;
end


% set nonzero offsets
joint_offset_map.l_arm_shx = -1.45;
joint_offset_map.l_arm_ely = 1.57;
joint_offset_map.r_arm_shx = 1.45;
joint_offset_map.r_arm_ely = 1.57;
joint_offset_map.l_leg_kny = 1.57;

joint_sign_map = getAtlasJointSignMap(state_frame, nq);

if ~isfield(joint_index_map,joint)
    error ('unknown joint name');
end

gains = getAtlasGains(input_frame);

% zero out force gains to start --- move to nominal joint position
gains.k_f_p = zeros(nu,1);
gains.ff_f_d = zeros(nu,1);
gains.ff_qd = zeros(nu,1);
ref_frame.updateGains(gains);

qdes = getQDesiredToMoveSingleJoint(joint, joint_index_map);
qdes(joint_index_map.(joint)) = joint_offset_map.(joint);

actuator_indices = getActuatedJoints(robot);

% move to desired pos
% atlasLinearMoveToPos(qdes,state_frame,ref_frame,actuator_indices,3);

disp('Ready to send input signal.');
% keyboard;

% set gains to user specified values
actuator_index = find(actuator_indices == joint_index_map.(joint));
gains.ff_const(actuator_index) = gains_for_joint_to_tune.ff_const;

switch control_mode
    case AtlasJointControlMode.FORCE
        % set force gains
        gains.k_f_p(actuator_index) = gains_for_joint_to_tune.k_f_p;
        gains.ff_f_d(actuator_index) = gains_for_joint_to_tune.ff_f_d;
        gains.ff_qd(actuator_index) = gains_for_joint_to_tune.ff_qd;
        
        % set joint position gains to 0
        gains.k_q_p(actuator_index) = 0;
        gains.k_q_i(actuator_index) = 0;
        gains.k_qd_p(actuator_index) = 0;
    case AtlasJointControlMode.POSITION
        % set force gains to 0
        gains.k_f_p(actuator_index) = 0;
        gains.ff_f_d(actuator_index) = 0;
        gains.ff_qd(actuator_index) = 0;
        
        % set joint position gains
        gains.k_q_p(actuator_index) = gains_for_joint_to_tune.k_q_p;
        gains.k_q_i(actuator_index) = gains_for_joint_to_tune.k_q_i;
        gains.k_qd_p(actuator_index) = gains_for_joint_to_tune.k_qd_p;
    otherwise
        error(['controlMode not recognized: ' control_mode]);
end

ref_frame.updateGains(gains);
udes = zeros(nu,1);

vals = joint_sign_map.(joint) * vals;
if control_mode == AtlasJointControlMode.POSITION
    vals = joint_offset_map.(joint) + vals;
end

if strcmp(signal,'zoh')
    input_traj = PPTrajectory(zoh(ts,vals));
elseif strcmp(signal,'foh')
    input_traj = PPTrajectory(foh(ts,vals));
elseif strcmp(signal,'chirp')
    offset = 0;
    if control_mode == AtlasJointControlMode.POSITION
        offset=joint_offset_map.(joint);
    end
    if zero_crossing
        input_traj = PPTrajectory(foh(ts, offset + amp*sin(ts.*freq*2*pi)));
    else
        input_traj = PPTrajectory(foh(ts, offset + joint_sign_map.(joint)*(0.5*amp - 0.5*amp*cos(ts.*freq*2*pi))));
    end
else
    error('unknown signal');
end

qdes=qdes(actuator_indices);
toffset = -1;
tt=-1;
T = ts(end);
while tt<T
    [x,t] = getNextMessage(state_frame,1);
    if ~isempty(x)
        if toffset==-1
            toffset=t;
        end
        tt=t-toffset;
        
        switch control_mode
            case AtlasJointControlMode.FORCE
                udes(actuator_index) = input_traj.eval(tt);
            case AtlasJointControlMode.POSITION
                qdes(actuator_index) = input_traj.eval(tt);
            otherwise
                error(['controlMode not recognized: ' control_mode]);
        end
        ref_frame.publish(t,[qdes;udes],'ATLAS_COMMAND');
    end
end

end

function gains = getGainsForJointToTune(control_mode)
gains.ff_const = 0.1;% <----
switch (control_mode)
    case AtlasJointControlMode.FORCE
        % force gains: only have an effect if controlMode==AtlasJointControlMode.TORQUE
        gains.k_f_p = 0.125;% <----
        gains.ff_f_d = 0.009;% <----
        gains.ff_qd = 0.0;% <----
    case AtlasJointControlMode.POSITION
        % position gains: only have an effect if controlMode==AtlasJointControlMode.POSITION
        gains.k_q_p = 0.0;% <----
        gains.k_q_i = 0.0;% <----
        gains.k_qd_p = 0.0;% <----
    otherwise
        error(['controlMode not recognized: ' control_mode]);
end
end

function checkGainRanges(gains, control_mode)
% check gain ranges --- TODO: make this more conservative
rangecheck(gains.ff_const,-10,10);
switch (control_mode)
    case AtlasJointControlMode.FORCE
        rangecheck(gains.k_f_p,0,1);
        rangecheck(gains.ff_f_d,0,1);
        rangecheck(gains.ff_qd,0,1);
    case AtlasJointControlMode.POSITION
        rangecheck(gains.k_q_p,0,50);
        rangecheck(gains.k_q_i,0,0.5);
        rangecheck(gains.k_qd_p,0,50);
    otherwise
        error(['controlMode not recognized: ' control_mode]);
end
end

function checkReferenceRanges(vals, control_mode)
switch control_mode
    case AtlasJointControlMode.FORCE
        %   rangecheck(vals,-70,70);
        %   if ~rangecheck(vals,-50,50)
        %     disp('Warning: about to command relatively high torque. Shift+F5 to cancel.');
        %     keyboard;
        %   end
    case AtlasJointControlMode.POSITION
        rangecheck(vals,-pi,pi);
        if ~rangecheck(vals,-1,1)
            disp('Warning: about to command relatively large position change. Shift+F5 to cancel.');
            keyboard;
        end
    otherwise
        error(['controlMode not recognized: ' control_mode]);
end
end
