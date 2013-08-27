function atlasMoveToHomePos(desired_neck_pitch)
%NOTEST

if nargin < 1
  desired_neck_pitch = 0;
end

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

nq = getNumDOF(r);
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
qstar = xstar(1:nq);
neck_idx = ~cellfun(@isempty,strfind(state_frame.coordinates(1:nq),'neck_ay'));
qstar(neck_idx) = desired_neck_pitch;

act_idx = getActuatedJoints(r);

disp('Moving to nominal pose (10 sec).');
atlasLinearMoveToPos(qstar,state_frame,input_frame,act_idx,4);

