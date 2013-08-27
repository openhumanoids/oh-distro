function atlasNeckControl(desired_neck_pitch)
%NOTEST
% simple script for commanding neck pitch angles---temporary for early
% testing

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

if nargin < 1
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  desired_neck_pitch = 0.2;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
else
  desired_neck_pitch = max(-0.2,min(1.0,desired_neck_pitch));
end

neck_idx = ~cellfun(@isempty,strfind(state_frame.coordinates,'neck_ay'));
act_idx = getActuatedJoints(r);

while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    qdes=x(1:getNumDOF(r));
    qdes(neck_idx) = desired_neck_pitch;
    break;
  end
end

atlasLinearMoveToPos(qdes,state_frame,input_frame,act_idx,4.0);

end
