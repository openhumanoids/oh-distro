function calibrateAtlasArmEncoders(runLCM)
%NOTEST

if nargin < 1
  runLCM = false;
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% must run this with encoders disabled in the state sync process
r = DRCAtlas();
nq = getNumPositions(r);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
ref_frame = drcFrames.AtlasPositionRef(r);

% extra frame contains raw encoder values
extra_frame = LCMCoordinateFrame('drcFrames.AtlasStateExtra',4*(nq-6),'x');
coder = drc.control.AtlasStateExtraCoder(nq-6);
extra_frame.setLCMCoder(JLCMCoder(coder));
extra_frame.subscribe('ATLAS_STATE_EXTRA');

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.getCoordinateNames{i}) = i;
end

act_idx = getActuatedJoints(r);

[jlmin,jlmax] = getJointLimits(r);
delta = 0.2; % [rad] amount to command beyond joint limits

% initial/final configuration
x0 = Point(state_frame);
x0.r_arm_shx = 1.45;
x0.l_arm_shx = -1.45;
x0 = double(x0);
q0 = x0(1:nq);


% calibration configuration: drive arm joints to limits
x_calib = Point(state_frame);
x_calib.r_arm_shz = jlmin(joint_index_map.r_arm_shz) - delta;
x_calib.r_arm_shx = jlmax(joint_index_map.r_arm_shx) + delta;
x_calib.r_arm_ely = jlmin(joint_index_map.r_arm_ely) - delta;
x_calib.r_arm_elx = jlmax(joint_index_map.r_arm_elx) + delta;
x_calib.r_arm_uwy = 0; % we dont calibrate these joints anymore
x_calib.r_arm_mwx = jlmin(joint_index_map.r_arm_elx) + delta; % just get the hand out of the way
x_calib.r_arm_lwy = 0;
x_calib.l_arm_shz = jlmax(joint_index_map.l_arm_shz) + delta;
x_calib.l_arm_shx = jlmin(joint_index_map.l_arm_shx) - delta;
x_calib.l_arm_ely = jlmin(joint_index_map.l_arm_ely) - delta;
x_calib.l_arm_elx = jlmin(joint_index_map.l_arm_elx) - delta;
x_calib.l_arm_uwy = 0;
x_calib.l_arm_mwx = jlmax(joint_index_map.l_arm_elx) - delta; % just get the hand out of the way
x_calib.l_arm_lwy = 0;
x_calib = double(x_calib);
q_calib = x_calib(1:nq);

% "correct" encoder readings at joint limits
calib_val = Point(state_frame);

calib_val.r_arm_shz = -0.8050;
calib_val.r_arm_shx =  1.5789;
calib_val.r_arm_ely = -0.0017;
calib_val.r_arm_elx = 0.0026;
calib_val.r_arm_uwy = 0;
calib_val.r_arm_mwx = 0;
calib_val.r_arm_lwy = 0;

calib_val.l_arm_shz = 0.7494;
calib_val.l_arm_shx = -1.5894;
calib_val.l_arm_ely = -9.8400e-04;
calib_val.l_arm_elx = -0.0548;
calib_val.l_arm_uwy = 0;
calib_val.l_arm_mwx = 0;
calib_val.l_arm_lwy = 0;

calib_val = double(calib_val);

behavior_pub = AtlasBehaviorModePublisher('ATLAS_BEHAVIOR_COMMAND');
behavior_lis = AtlasBehaviorModeListener('ATLAS_STATUS');

lc = lcm.lcm.LCM.getSingleton();
monitor = drake.util.MessageMonitor(drc.utime_t,'utime');
lc.subscribe('CALIBRATE_ARM_ENCODERS',monitor);  
if runLCM % wait for LCM trigger, then run
  while true
    pause(0.25);
    x = monitor.getNextMessage(10);
    if ~isempty(x)
      moveAndWriteCalibration();
    end
  end
else
  moveAndWriteCalibration();
end

  function moveAndWriteCalibration

    send_controller_state(lc, 'DUMMY');
    switch_back_to_freeze = false;
    % get current atlas behavior
    pause(0.25);
    behavior = behavior_lis.getMessage();
    if behavior == AtlasBehaviorModeListener.BEHAVIOR_FREEZE
      switch_back_to_freeze = true;
      % switch to user mode
      d.utime = 0;
      d.command = 'user';
      behavior_pub.publish(d);
      pause(5.0);
  
      % gross, have to turn the controller off again after user transition
      msg = drc.utime_t();
      msg.utime = 0; % enable with positive utime
      lc.publish('CALIBRATE_ARM_ENCODERS',msg);
      pause(5.0);
      monitor.getNextMessage(10);
    end
    
    msg = drc.utime_t();
    msg.utime = -1; % disable with negative utime
    lc.publish('ENABLE_ENCODERS',msg);
    pause(0.1);

    % move to initial pos
    atlasLinearMoveToPos(q0,state_frame,ref_frame,act_idx,5);

    % move to calib pos
    atlasLinearMoveToPos(q_calib,state_frame,ref_frame,act_idx,7);

    pause(0.1);
    % record encoder values
    [ex,~] = extra_frame.getMessage();

    % move to initial pos again
    atlasLinearMoveToPos(q0,state_frame,ref_frame,act_idx,5);

    % display encoder offsets 
    ex = ex(1:(nq-6)); % just grab state off the robot
    length(calib_val(r.stateToBDIInd)) 
    length(ex)
    enc_diff = calib_val(r.stateToBDIInd) - ex;

    % get original offsets from config
    client = BotParam();
    orig_indices = client.get('control.encoder_offsets.index')+1;
    orig_offsets = client.get('control.encoder_offsets.value');
    
    % note: using BDI's order, +1 for matlab
    JOINT_L_ARM_SHZ   = 17;
    JOINT_L_ARM_SHX   = 18;
    JOINT_L_ARM_ELY   = 19;
    JOINT_L_ARM_ELX   = 20;
    JOINT_L_ARM_UWY   = 21;
    JOINT_L_ARM_MWX   = 22;
    JOINT_L_ARM_LWY   = 23;
    JOINT_R_ARM_SHZ   = 24;
    JOINT_R_ARM_SHX   = 25;
    JOINT_R_ARM_ELY   = 26;
    JOINT_R_ARM_ELX   = 27;
    JOINT_R_ARM_UWY   = 28;
    JOINT_R_ARM_MWX   = 29;
    JOINT_R_ARM_LWY   = 30;

    cur_indices = [
        JOINT_L_ARM_SHZ
        JOINT_L_ARM_SHX
        JOINT_L_ARM_ELY
        JOINT_L_ARM_ELX
        JOINT_R_ARM_SHZ
        JOINT_R_ARM_SHX
        JOINT_R_ARM_ELY
        JOINT_R_ARM_ELX
    ];
    cur_offsets = enc_diff(cur_indices);

    % replace existing offsets
    final_indices = orig_indices(:);
    final_offsets = orig_offsets(:);
    for k = 1:numel(cur_indices)
      idx = find(final_indices==cur_indices(k));
      if ~isempty(idx)
        final_offsets(idx) = cur_offsets(k);
      else
        final_indices = [final_indices; cur_indices(k)];
        final_offsets = [final_offsets; cur_offsets(k)];
      end
    end
    
    msg = bot_param.set_t();
    msg.utime = bot_timestamp_now();
    msg.sequence_number = bot_param_get_seqno();
    msg.server_id = bot_param_get_server_id();

    % convert values to strings
    index_strings = strtrim(cellstr(num2str(final_indices-1)))';
    offset_strings = strtrim(cellstr(num2str(final_offsets)))';
    index_string = strjoin(index_strings,',');
    offset_string = strjoin(offset_strings,',');
    
    joint_ind = bot_param.entry_t();
    joint_ind.is_array = true;
    joint_ind.key = 'control.encoder_offsets.index';
    joint_ind.value = index_string;
    
    offsets = bot_param.entry_t();
    offsets.is_array = true;
    offsets.key = 'control.encoder_offsets.value';
    offsets.value = offset_string;
    
    msg.numEntries = 2;
    msg.entries = javaArray('bot_param.entry_t', msg.numEntries);
    msg.entries(1) = joint_ind;
    msg.entries(2) = offsets;
    lc.publish('PARAM_SET',msg);
    disp('Arm encoder calibration completed.');
    send_status(1, 0, 0, 'Arm encoder calibration completed.');

    msg = drc.utime_t();
    msg.utime = 0;
    lc.publish('REFRESH_ENCODER_OFFSETS',msg);

    msg = drc.utime_t();
    msg.utime = 1; % enable with positive utime
    lc.publish('ENABLE_ENCODERS',msg);
    
    send_controller_state(lc, 'DUMMY');
    if switch_back_to_freeze      
      % switch to user mode
      d.utime = 0;
      d.command = 'freeze';
      behavior_pub.publish(d);
      pause(0.5);
    end

  end


  exit;

end

function send_controller_state(lc, state_name)
  msg = drc.controller_status_t();
  msg.utime = bot_timestamp_now();
  msg.state = msg.(state_name);
  lc.publish('CONTROLLER_STATUS', msg);
end
