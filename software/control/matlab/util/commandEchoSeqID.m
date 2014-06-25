
tmp_state = TEMPRobotStateListener('EST_ROBOT_STATE');
tmp_cmd = TEMPAtlasCommandPublisher('ATLAS_COMMAND');

while 1
  [seq_id,t] = getNextMessage(tmp_state,5);
  if ~isempty(seq_id)
    pause(3.5/1000)
    publish(tmp_cmd,t,seq_id);
  end
end