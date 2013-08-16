function atlasNeckControl(desired_neck_pitch)
%NOTEST
% simple script for commanding neck pitch angles---temporary for early
% testing

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
input_frame = AtlasPositionRef(r);

if nargin < 1
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  desired_neck_pitch = 0.75;
  %%%%%%%%%%%%%%%%%%%%%%%%%%%
else
  desired_neck_pitch = max(-0.2,min(1.0,desired_neck_pitch));
end

qdes = zeros(getNumInputs(r),1);
neck_idx = ~cellfun(@isempty,strfind(input_frame.coordinates,'neck_ay'));
qdes(neck_idx) = desired_neck_pitch;
input_frame.publish(0,qdes,'ATLAS_COMMAND');

end
