% the loop test uses the robot state and atlas command
% coders that are (partly) implemented in c++
% other coders are typically all in Java

% load robot model
r = Atlas();

% setup frames
state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
udes = zeros(getNumInputs(r),1);

xx =eye(300,300);
y = zeros(300,300);
while 1
  [x,t] = getNextMessage(state_frame,5);
  if ~isempty(x)
    y= inv(xx) ;
    input_frame.publish(t,udes,'ATLAS_COMMAND');
  end
end
