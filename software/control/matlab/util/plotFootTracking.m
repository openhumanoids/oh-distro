function plotFootTracking

% load robot model
r = Atlas();
state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');

load('walking_ctrl_data.mat');


[t_x,x_data,t_u,u_data,state_frame,input_frame] = parseAtlasLog(r,'/home/drc/logs/raw/lcmlog-2014-07-25.60');

nq = r.getNumDOF();


N = length(t_u);
lfoot_pos = zeros(6,N);
rfoot_pos = zeros(6,N);

lfoot_idx = findLinkInd(r,'l_foot');
rfoot_idx = findLinkInd(r,'r_foot');

for i=1:N
  [~, x_idx] = min(abs(t_u(i) - t_x));
  q = x_data(1:nq,x_idx);
  qd = x_data(nq+(1:nq),x_idx);
  kinsol = r.doKinematics(q,true);
  lfoot_pos(:,i) = forwardKin(r,kinsol,lfoot_idx,[0;0;0],1);
  rfoot_pos(:,i) = forwardKin(r,kinsol,rfoot_idx,[0;0;0],1);
  
end

lfoot_link_con_idx = [walking_ctrl_data.link_constraints.link_ndx]==lfoot_idx;
rfoot_link_con_idx = [walking_ctrl_data.link_constraints.link_ndx]==rfoot_idx;

lfoot_traj_des = walking_ctrl_data.link_constraints(lfoot_link_con_idx).traj;
rfoot_traj_des = walking_ctrl_data.link_constraints(rfoot_link_con_idx).traj;


% plot_dim('l_foot X',lfoot_traj_des,lfoot_pos,1,t_u);
% plot_dim('l_foot Y',lfoot_traj_des,lfoot_pos,2,t_u);
% plot_dim('l_foot Z',lfoot_traj_des,lfoot_pos,3,t_u);
% plot_dim('l_foot Roll',lfoot_traj_des,lfoot_pos,4,t_u);
% plot_dim('l_foot Pitch',lfoot_traj_des,lfoot_pos,5,t_u);
% plot_dim('l_foot Yaw',lfoot_traj_des,lfoot_pos,6,t_u);
% 
plot_dim('r_foot X',rfoot_traj_des,rfoot_pos,1,t_u);
plot_dim('r_foot Y',rfoot_traj_des,rfoot_pos,2,t_u);
plot_dim('r_foot Z',rfoot_traj_des,rfoot_pos,3,t_u);
plot_dim('r_foot Roll',rfoot_traj_des,rfoot_pos,4,t_u);
plot_dim('r_foot Pitch',rfoot_traj_des,rfoot_pos,5,t_u);
plot_dim('r_foot Yaw',rfoot_traj_des,rfoot_pos,6,t_u);


end

function plot_dim(title_str,foot_des_traj,foot_pos_matrix,index,t_u)

figure(index);
fnplt(foot_des_traj(index));
hold on;
plot(t_u-t_u(1),foot_pos_matrix(index,:),'r');
title(title_str);
hold off;

end