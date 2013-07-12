function tunePDPinned(dofnum,amplitude,tstep,T)
%NOTEST

options.floating = false;
options.dt = 0.001;
%r = Atlas('../urdf/atlas_minimal_contact.urdf',options);
r = Atlas('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf',options);
% set initial state to fixed point
load('../data/atlas_pinned_config.mat');

[jl_min,jl_max] = r.getJointLimits();
theta_mid = jl_min(dofnum) + (jl_max(dofnum)-jl_min(dofnum))/2;
xstar(dofnum) = theta_mid;
%xstar(25) = 1.5; % bend right knee
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.05;

[kp,kd] = getPDGainsPinned(r); 
pd = pdcontrol(r,kp,kd);

q_des = [];
q_des_t = xstar(1:r.getNumDOF);
for t=0:tstep:T
  q_des_t(dofnum) = theta_mid + amplitude*sin(t^2/pi);
  q_des = [q_des q_des_t];
end

act_idx = r.getActuatedJoints();
theta_des = q_des(act_idx,:);

des_traj = setOutputFrame(PPTrajectory(zoh(0:tstep:T,theta_des)),pd.getInputFrame());

figure(11);
dtr = des_traj.eval(0:options.dt:T);
plot(0:options.dt:T,dtr(find(act_idx==dofnum),:));


sys = cascade(des_traj,pd); 
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
traj = simulate(sys,[0 T]);
playback(v,traj,struct('slider',true));

xtr = traj.eval(0:options.dt:T);
thtr = xtr(dofnum,:);

figure(11);
hold on;
plot(0:options.dt:T,thtr,'r');
title(r.getStateFrame.coordinates(dofnum));
hold off;

end




