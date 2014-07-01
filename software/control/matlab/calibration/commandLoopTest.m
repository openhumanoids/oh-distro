
tmp_state = LoopTestListener('EST_ROBOT_STATE');
tmp_cmd = LoopTestPublisher('ATLAS_COMMAND');

while 1
  [t] = getNextMessage(tmp_state,5);
  if ~isempty(t)
    pause(3.5/1000)
    publish(tmp_cmd,t);
  end
end
