function executeReachPlan(r,state_frame,xtraj,ts)
T = max(ts);
x0 = getInitialState(r); 
ind = getActuatedJoints(r);

qd_frame =  PinnedAtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(spline(ts,xtraj(ind,:))),qd_frame);

% execute plan
disp('Executing plan...');
t=0;
t_offset = -1;
%texec=[];
while t <= T
  [x,tsim] = getNextMessage(state_frame,50);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    disp(t);
    %texec(end+1)=t;
    qd=des_traj.eval(t);
    qd_frame.publish(t,qd,'JOINT_COMMANDS');
  end
end

% [N,X]=hist(diff(texec),20);plot(X,N,'b.-')
