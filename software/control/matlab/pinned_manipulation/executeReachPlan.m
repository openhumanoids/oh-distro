function executeReachPlan(r,state_frame,xtraj)
T = 5.0; % seconds, hard coded for now
dt = 1;
ts = 0:dt:T; % plan timesteps

x0 = getInitialState(r); 
% lame for now
ind = getActuatedJoints(r);

%  WTF was this?
% for i=1:size(x0,2);
%   q = xtraj(i,:);
%   q_d = q(ind);
% end
% des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);

qd_frame =  AtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(spline(ts,xtraj(ind,:))),qd_frame);

% execute plan
disp('Executing plan...');
t=0;
t_offset = -1;
while t <= T
  [x,tsim] = getNextMessage(state_frame,100);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    disp(t);
    %t=t+1;
    qd=des_traj.eval(t);
    qd_frame.publish(t,qd,'JOINT_COMMANDS');
  end
end
