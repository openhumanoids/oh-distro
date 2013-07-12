function logFallingTraj
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

outs(1).system=2;
outs(1).output=1;
sys = mimoCascade(utraj,r,[],[],outs);
clear outs;

v = r.constructVisualizer;
v.display_dt = 0.01;

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
outs(1).system=1;
outs(1).output=1;
sys = mimoCascade(sys,v,[],[],outs);
warning(S);
xtraj = simulate(sys,[0 T],xstar);
playback(v,xtraj,struct('slider',true));

save('drake_xtraj.mat','xtraj');

end