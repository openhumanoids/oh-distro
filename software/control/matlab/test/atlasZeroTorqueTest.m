function atlasZeroTorqueTest
%NOTEST
% sends zero torque refs to the robot

% 8-15-2013 DO NOT RUN THIS IN HIGH PRESSURE MODE --sk

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);

nu = getNumInputs(r);

u=zeros(nu,1);
while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    % do zero torque control

    input_frame.publish(t,u,'ATLAS_COMMAND');
  end
end

end
