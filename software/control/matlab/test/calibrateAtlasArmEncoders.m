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
delta = 0.15; % [rad] amount to command beyond joint limits

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
atlasLinearMoveToPos(q0,state_frame,ref_frame,act_idx,3);

% move to calib pos
atlasLinearMoveToPos(q_calib,state_frame,ref_frame,act_idx,3);

% record encoder values
[ex,~] = extra_frame.getMessage();

% move to initial pos again
atlasLinearMoveToPos(q0,state_frame,ref_frame,act_idx,3);

% display encoder offsets 
ex = ex(1:2*nq); % just grab state off the robot
enc_diff = calib_val - ex;

fprintf('l_arm_usy offset: %2.4f\n',enc_diff(joint_index_map.l_arm_usy));
fprintf('l_arm_shx offset: %2.4f\n',enc_diff(joint_index_map.l_arm_shx));
fprintf('l_arm_ely offset: %2.4f\n',enc_diff(joint_index_map.l_arm_ely));
fprintf('l_arm_elx offset: %2.4f\n',enc_diff(joint_index_map.l_arm_elx));
fprintf('l_arm_uwy offset: %2.4f\n',enc_diff(joint_index_map.l_arm_uwy));
fprintf('l_arm_mwx offset: %2.4f\n\n',enc_diff(joint_index_map.l_arm_mwx));

fprintf('r_arm_usy offset: %2.4f\n',enc_diff(joint_index_map.r_arm_usy));
fprintf('r_arm_shx offset: %2.4f\n',enc_diff(joint_index_map.r_arm_shx));
fprintf('r_arm_ely offset: %2.4f\n',enc_diff(joint_index_map.r_arm_ely));
fprintf('r_arm_elx offset: %2.4f\n',enc_diff(joint_index_map.r_arm_elx));
fprintf('r_arm_uwy offset: %2.4f\n',enc_diff(joint_index_map.r_arm_uwy));
fprintf('r_arm_mwx offset: %2.4f\n',enc_diff(joint_index_map.r_arm_mwx));

end
