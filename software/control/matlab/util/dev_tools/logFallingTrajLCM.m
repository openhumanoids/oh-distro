function logFallingTrajLCM
%NOTEST

options.floating = true;
options.dt = 0.001;

r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

xstar = zeros(getNumStates(r),1);
xstar(3) = 1;
xstar(5) = 0.4;
xstar(6) = 1.57;
r = setInitialState(r,xstar);

state_frame = getStateFrame(r);
state_frame.subscribe('TRUE_ROBOT_STATE');

joint_names = r.getInputFrame.coordinates;
% shx_joints = find(~cellfun(@isempty,strfind(joint_names,'shx')) + ...
%                   ~cellfun(@isempty,strfind(joint_names,'lbz')) + ...
%                   ~cellfun(@isempty,strfind(joint_names,'kny')));
shx_joints = find(~cellfun(@isempty,strfind(joint_names,'mby')));

T = 0.7;
ts = 0:0.01:T;
utraj = zeros(getNumInputs(r),length(ts));
% for i=1:length(ts)
%   t=ts(i);
%   utraj(shx_joints,i) = 100*sin(2*t*pi)*ones(length(shx_joints),1);
% end

utraj = setOutputFrame(PPTrajectory(zoh(ts,utraj)),getInputFrame(r));

input_frame = getInputFrame(r);
t=0;
t_offset = -1;
T = ts(end);
disp('waiting...');
tts = [];
xtraj = [];
while t <= T
  [x,tsim] = getNextMessage(state_frame,1);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    tts = [tts t];
    xtraj = [xtraj x];
    input_frame.publish(t,utraj.eval(t),'JOINT_COMMANDS');
  end
end
disp('done.');

xtraj = setOutputFrame(PPTrajectory(zoh(tts,xtraj)),getStateFrame(r));

save('gazebo_xtraj.mat','xtraj');
end