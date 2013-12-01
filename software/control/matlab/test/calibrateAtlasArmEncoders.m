function calibrateAtlasArmEncoders
%NOTEST

% must run this with encoders disabled in the state sync process
r = Atlas();
nq = getNumDOF(r);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
ref_frame = AtlasPositionRef(r);

% extra frame contains raw encoder values
extra_frame = LCMCoordinateFrameWCoder('AtlasStateExtra',4*28,'x');
coder = drc.control.AtlasStateExtraCoder(28);
extra_frame.setLCMCoder(JLCMCoder(coder));
extra_frame.subscribe('ATLAS_STATE_EXTRA');

joint_index_map = struct(); % maps joint names to indices
for i=1:nq
  joint_index_map.(state_frame.coordinates{i}) = i;
end

act_idx = getActuatedJoints(r);

[jlmin,jlmax] = getJointLimits(r);
delta = 0.25; % [rad] amount to command beyond joint limits

% initial/final configuration
x0 = Point(state_frame);
x0.r_arm_shx = 1.45;
x0.l_arm_shx = -1.45;
x0 = double(x0);
q0 = x0(1:nq);

% calibration configuration: drive arm joints to limits
x_calib = Point(state_frame);
x_calib.r_arm_usy = jlmax(joint_index_map.r_arm_usy) + delta;
x_calib.r_arm_shx = jlmax(joint_index_map.r_arm_shx) + delta;
x_calib.r_arm_ely = jlmin(joint_index_map.r_arm_ely) - delta;
x_calib.r_arm_elx = jlmax(joint_index_map.r_arm_elx) + delta;
x_calib.r_arm_uwy = jlmin(joint_index_map.r_arm_uwy) - delta;
x_calib.r_arm_mwx = jlmin(joint_index_map.r_arm_mwx) - delta;
x_calib.l_arm_usy = jlmax(joint_index_map.l_arm_usy) + delta;
x_calib.l_arm_shx = jlmin(joint_index_map.l_arm_shx) - delta;
x_calib.l_arm_ely = jlmin(joint_index_map.l_arm_ely) - delta;
x_calib.l_arm_elx = jlmin(joint_index_map.l_arm_elx) - delta;
x_calib.l_arm_uwy = jlmin(joint_index_map.l_arm_uwy) - delta;
x_calib.l_arm_mwx = jlmax(joint_index_map.l_arm_mwx) + delta;
x_calib = double(x_calib);
q_calib = x_calib(1:nq);

% "correct" encoder readings at joint limits
calib_val = Point(state_frame);
calib_val.r_arm_usy =  0.79804;
calib_val.r_arm_shx =  1.57022;
calib_val.r_arm_ely = -0.01648;
calib_val.r_arm_elx = -0.00297;
calib_val.r_arm_uwy =  0.02374;
calib_val.r_arm_mwx = -1.22977;
calib_val.l_arm_usy =  0.79858;
calib_val.l_arm_shx = -1.58616;
calib_val.l_arm_ely = -0.04079;
calib_val.l_arm_elx =  0.00481;
calib_val.l_arm_uwy =  0.02515;
calib_val.l_arm_mwx =  1.16325;
calib_val = double(calib_val);

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
ex = ex(1:28); % just grab state off the robot
enc_diff = calib_val(bdiInd()) - ex;

% note: using BDI's order (extra message uses this)
JOINT_L_ARM_USY   = 17;
JOINT_L_ARM_SHX   = 18;
JOINT_L_ARM_ELY   = 19;
JOINT_L_ARM_ELX   = 20;
JOINT_L_ARM_UWY   = 21;
JOINT_L_ARM_MWX   = 22;
JOINT_R_ARM_USY   = 23;
JOINT_R_ARM_SHX   = 24;
JOINT_R_ARM_ELY   = 25;
JOINT_R_ARM_ELX   = 26;
JOINT_R_ARM_UWY   = 27;
JOINT_R_ARM_MWX   = 28;

fprintf('\nPaste the following code into state_sync.cpp in the area highlighted at the top:\n\n');
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_USY] = %2.4f;\n',enc_diff(JOINT_R_ARM_USY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_SHX] = %2.4f;\n',enc_diff(JOINT_R_ARM_SHX));
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELY] = %2.4f;\n',enc_diff(JOINT_R_ARM_ELY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_ELX] = %2.4f;\n',enc_diff(JOINT_R_ARM_ELX));
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_UWY] = %2.4f;\n',enc_diff(JOINT_R_ARM_UWY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_R_ARM_MWX] = %2.4f;\n\n',enc_diff(JOINT_R_ARM_MWX));

fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_USY] = %2.4f;\n',enc_diff(JOINT_L_ARM_USY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_SHX] = %2.4f;\n',enc_diff(JOINT_L_ARM_SHX));
fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_ELY] = %2.4f;\n',enc_diff(JOINT_L_ARM_ELY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_ELX] = %2.4f;\n',enc_diff(JOINT_L_ARM_ELX));
fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_UWY] = %2.4f;\n',enc_diff(JOINT_L_ARM_UWY));
fprintf('encoder_joint_offsets_[Atlas::JOINT_L_ARM_MWX] = %2.4f;\n',enc_diff(JOINT_L_ARM_MWX));

fprintf('\n then recompile and restart the state sync process: drc-state-sync -b -e\n\n');

end
