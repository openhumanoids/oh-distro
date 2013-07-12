function x = getRobotState(channel)
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe(channel);
while true
  [x,~] = getNextMessage(state_frame,10);
  if (~isempty(x))
    break
  end
end
