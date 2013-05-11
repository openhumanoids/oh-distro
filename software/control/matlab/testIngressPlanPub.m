% function [t_test,com_test,com_nom] = testIngressPlanPub(scale_t,foot_support_qs)
function testIngressPlanPub(scale_t,foot_support_qs)

addpath(fullfile(pwd,'frames'));

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

load('data/atlas_fp.mat');
load('data/aa_step_in.mat');
t_qs_breaks = t_qs_breaks*scale_t;

if size(foot_support_qs,1) == 35
  foot_support_qs(1:5,:) = [];
end

l_foot_ind = r.findLinkInd('l_foot');
r_foot_ind = r.findLinkInd('r_foot');
not_feet_ind = (1:r.getNumBodies ~= l_foot_ind) &...
               (1:r.getNumBodies ~= r_foot_ind);
foot_support_qs(not_feet_ind,:)=0;

state_frame = getStateFrame(r);
state_frame.subscribe('TRUE_ROBOT_STATE');

while true
  [x,~] = getNextMessage(state_frame,10);
  if (~isempty(x))
    q0=x(1:r.getNumDOF());
    break
  end
end

nt = numel(t_qs_breaks);
nq = getNumDOF(r);
q_nom = xstar(1:nq);
q0_nom = q_qs_plan(1:nq,1);
x0_nom = [q0,zeros(size(q0))];

ref_link = r.findLink(ref_link_str);

%kinsol = doKinematics(r,q0_nom);
%r_foot_xyz_nom = forwardKin(r,kinsol,r_foot_body,[0;0;0]);
%fprintf(['Nominal right foot position:\n\t' ...
            %'x: %5.3f\n\t' ...
            %'y: %5.3f\n'], r_foot_xyz_nom(1:2))
kinsol = doKinematics(r,q0,false,false);
wTf = ref_link.T;

com_qs_plan = homogTransMult(wTf,com_qs_plan);
for i = 1:nt
  fTr_i = [ [rpy2rotmat(q_qs_plan(4:6,i)); zeros(1,3)], [q_qs_plan(1:3,i); 1] ];
  wTr_i = wTf*fTr_i;
  q_qs_plan(1:6,i) = [wTr_i(1:3,4); rotmat2rpy(wTr_i(1:3,1:3))];
end

%r_foot_xyz = forwardKin(r,kinsol,r_foot_body,[0;0;0]);
%fprintf(['Actual right foot position:\n\t' ...
            %'x: %5.3f\n\t' ...
            %'y: %5.3f\n'], r_foot_xyz(1:2))

%xy_offset = r_foot_xyz(1:2) - r_foot_xyz_nom(1:2);

%fprintf(['Adjusting trajectories in x and y:\n\t' ...
            %'x offset: %5.3f\n\t' ...
            %'y offset: %5.3f\n'], xy_offset);

%q_qs_plan(1:2,:) = q_qs_plan(1:2,:) + repmat(xy_offset,1,nt);
%com_qs_plan(1:2,:) = com_qs_plan(1:2,:) + repmat(xy_offset,1,nt);
% com = getCOM(r,kinsol);

% create desired joint trajectory
% cost = Point(r.getStateFrame,1);
% % cost.base_x = 0;
% % cost.base_y = 0;
% % cost.base_z = 0;
% % cost.base_roll = 1000;
% % cost.base_pitch = 1000;
% % cost.base_yaw = 0;
% cost.back_mby = 100;
% cost.back_ubx = 100;
% cost = double(cost);
% options = struct();
% options.Q = diag(cost(1:r.getNumDOF));
%   
% rhand_body = r.findLink('r_hand');
% lhand_body = r.findLink('l_hand');
% rhand_pos = forwardKin(r,kinsol,rhand_body,[0;0;0],false);
% lhand_pos = forwardKin(r,kinsol,lhand_body,[0;0;0],false);
% 
% % pelvis_body = r.findLink('pelvis');
% % pelvis_pos = forwardKin(r,kinsol,pelvis_body,[0;0;0],true);  
% 
% % q_nom(1:6) = pelvis_pos;
% options.q_nom = q_nom;
% 
% rhand_goal = [-0.4; -0.25; 1.35];
% lhand_goal = [0.4; -0.25; 1.05];
% 
% rfoot_body = r.findLink('r_foot');
% lfoot_body = r.findLink('l_foot');
% rfoot_pos = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
% lfoot_pos = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);
% 
% % time spacing of samples for IK
% T = 4;
% ts = 0:0.1:T;
% 
% rhand_traj = PPTrajectory(spline([0 T],[rhand_pos rhand_goal]));
% lhand_traj = PPTrajectory(spline([0 T],[lhand_pos lhand_goal]));
% 
% for i=1:length(ts)
%   t = ts(i);
%   if (i>1)
%     q(:,i) = approximateIK(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
%       lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
%       lhand_body,[0;0;0],lhand_traj.eval(t),options);
% %    q(:,i) = inverseKin(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
% %      lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
% %      lhand_body,[0;0;0],lhand_traj.eval(t),options);
%   else
%     q = q0;
%   end
% end
qtraj = PPTrajectory(spline(t_qs_breaks,q_qs_plan));
comtraj = PPTrajectory(spline(t_qs_breaks,com_qs_plan(1:2,:)));
htraj = PPTrajectory(spline(t_qs_breaks,com_qs_plan(3,:)));
foot_support=PPTrajectory(zoh(t_qs_breaks,foot_support_qs));

Q = 10*eye(4);
R = 0.001*eye(2);
comgoal = com_qs_plan(1:2,end);
ltisys = LinearSystem([zeros(2),eye(2); zeros(2,4)],[zeros(2); eye(2)],[],[],[],[]);
[~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),Q,R);

% compute TVLQR
options.tspan = linspace(comtraj.tspan(1),comtraj.tspan(2),10);
options.sqrtmethod = false;
x0traj = setOutputFrame([comtraj;fnder(comtraj)],ltisys.getStateFrame);
u0traj = setOutputFrame(ConstantTrajectory([0;0]),ltisys.getInputFrame);
S = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
warning(S);
[~,V] = tvlqr(ltisys,x0traj,u0traj,Q,R,V,options);

% Simple PD parameters
Kp = [];
data = struct('qtraj',qtraj,'comtraj',comtraj,...
      'zmptraj',[],...
      'supptraj',foot_support,'htraj',[],'hddtraj',[],...
      'S',V.S,'s1',V.s1,'lfoottraj',[],'rfoottraj',[]);

pub=WalkingPlanPublisher('QUASISTATIC_ROBOT_PLAN'); % hijacking walking plan type for now
still_going = true;
while still_going
  pub.publish(0,data);
  in = input('Re-publish? (y/N): ')
  if isempty(in) || strcmpi(in,'n')
    still_going = false;
  end
end

% [x,t] = getNextMessage(state_frame,10);
% q_test = x(1:r.getNumDOF());
% t_start = t;
% t_test = t;
% 
% for i = 1:2000
%   disp('Get state');
%   [x,t] = getNextMessage(state_frame,10);
%   if (~isempty(x))
%     q_test = [q_test, x(1:r.getNumDOF())];
%     t_test = [t_test, t];
%   end
% end
% com_test = zeros(3,size(q_test,2));
% com_nom = zeros(2,size(q_test,2));
% for i = 1:size(q_test,2)
%   kinsol = doKinematics(r,q_test(:,i));
%   com_test(:,i) = getCOM(r,kinsol);
%   com_nom(:,i) = comtraj.eval(t_test(i));
% end
% com_test = com_test(1:2,:);
