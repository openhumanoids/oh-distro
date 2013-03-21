function tunePDPinnedGazebo(dofnum,amplitude,tstep,T)
%NOTEST

options.floating = false;
options.dt = 0.001;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('../data/atlas_pinned_config.mat');

[jl_min,jl_max] = r.getJointLimits();
theta_mid = jl_min(dofnum) + (jl_max(dofnum)-jl_min(dofnum))/2;
xstar(dofnum) = theta_mid;
%xstar(25) = 1.5; % bend right knee
%xstar(6) = 0; % left arm out
r = r.setInitialState(xstar);

% set initial conditions in gazebo
state_frame = r.getStateFrame();
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
state_frame.subscribe('EST_ROBOT_STATE');

q_des = [];
q_des_t = xstar(1:r.getNumDOF);
for t=0:tstep:T
  q_des_t(dofnum) = theta_mid + amplitude*sin(t^2/pi);
  q_des = [q_des q_des_t];
end

act_idx = r.getActuatedJoints();
theta_des = q_des(act_idx,:);

qd_frame = AtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(zoh(0:tstep:T,theta_des)),qd_frame);

t=0;
t_offset = -1;
ts = [];
dtr = [];
xtr = [];
disp('Ready...');
while t <= T
  [x,tsim] = getNextMessage(state_frame,1);
  if (~isempty(x) && (t_offset == -1 || t <= T))
    if (t_offset == -1)
      t_offset = tsim;
    end
    t=tsim-t_offset;
    qd=des_traj.eval(t);
    qd_frame.publish(t,qd,'JOINT_COMMANDS');
    dtr = [dtr qd];
    xtr = [xtr x];
    ts = [ts t];
  end
end

figure(11);
hold on;
plot(ts,xtr(dofnum,:),'r');
plot(ts,dtr(find(act_idx==dofnum),:));
title(r.getStateFrame.coordinates(dofnum));
hold off;

end




