function tunePDStandingGazebo(xyz_select,amplitude,timestep,T)
%NOTEST
% xyz_select \in {1,2,3}
% e.g., amp=0.55, timestep=0.2, T = 12

options.floating = true;
options.dt = 0.001;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('../data/atlas_fp3.mat');
xstar(3) = xstar(3)-0.002;
r = r.setInitialState(xstar);

% set initial conditions in gazebo
state_frame = r.getStateFrame();
state_frame.publish(0,xstar,'SET_ROBOT_CONFIG');
state_frame.subscribe('EST_ROBOT_STATE');

x0 = r.getInitialState(); 
q0 = x0(1:getNumDOF(r));

% get foot positions
kinsol = doKinematics(r,q0);
rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');

rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0]);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0]);

gc = r.contactPositions(q0);

% compute desired COM projection
% assumes minimal contact model for now
% k = convhull(gc(1:2,1:4)');
% lfootcen = [mean(gc(1:2,k),2);0];
% k = convhull(gc(1:2,5:8)');
% rfootcen = [mean(gc(1:2,4+k),2);0];
k = convhull(gc(1:2,:)');
com0 = getCOM(r,x0(1:getNumDOF(r)));
midfoot = com0;%[mean(gc(1:2,k),2);com0(3)];

com_des = [];
com_des_t = com0;
ts=0:timestep:T;
for i=1:length(ts)
  t = ts(i);
  com_des_t(xyz_select) = midfoot(xyz_select) + amplitude*sin(t^2/(5*pi));
  com_des = [com_des com_des_t];
end
comgoal = setOutputFrame(PPTrajectory(zoh(0:timestep:T,com_des)),AtlasCOM(r));

ind = getActuatedJoints(r);
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 10000;
cost.pelvis_pitch = 10000;
cost.pelvis_yaw = 10000;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
ikoptions = struct();
ikoptions.Q = diag(cost(1:r.getNumDOF));
ikoptions.q_nom = q0;
  
for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = inverseKin(r,q(:,i-1),0,comgoal.eval(t),rfoot_body,[rfoot0;0;0;0],lfoot_body,[lfoot0;0;0;0],ikoptions);
  else
    q = q0;
  end
  q_d(:,i) = q(ind,i);
end
qd_frame = AtlasPositionRef(r);
des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);

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

% plot COM trajectory tracking
comtraj_ax = [];
for i=1:length(ts)
  cm = getCOM(r,xtr(1:r.getNumDOF(),i));
  comtraj_ax = [comtraj_ax cm(xyz_select)];
end

figure(11);
hold on;
comtr = comgoal.eval(ts);
plot(ts,comtr(xyz_select,:));
plot(ts,comtraj_ax,'r');
title(strcat('COM tracking, axis = ',num2str(xyz_select)));
hold off;

act_idx = r.getActuatedJoints();
for j=[1:3,10:15]
  figure(17+j);
  hold on;
  plot(ts,dtr(j,:),'b');
  plot(ts,xtr(act_idx(j),:),'r');
  title(AtlasPositionRef(r).coordinates(j));
  hold off;
end



end
