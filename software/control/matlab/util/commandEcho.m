% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);

% setup frames
state_frame = AtlasState(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);

nu = getNumInputs(r);

udes = zeros(nu,1);

while 1
  [x,t] = getNextMessage(state_frame,5);
  if ~isempty(x)
    pause(4.5/1000)
    input_frame.publish(t,udes,'ATLAS_COMMAND');
  end
end