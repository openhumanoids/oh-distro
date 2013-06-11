function testIngressPlanPub(scale_t,foot_support_qs,filename)

addpath(fullfile(pwd,'frames'));

options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);

load('data/atlas_fp.mat');
load(filename);
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
state_frame.subscribe('EST_ROBOT_STATE');

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

kinsol = doKinematics(r,q0,false,false);
wTf = ref_link.T;

com_qs_plan = homogTransMult(wTf,com_qs_plan);
for i = 1:nt
  fTr_i = [ [rpy2rotmat(q_qs_plan(4:6,i)); zeros(1,3)], [q_qs_plan(1:3,i); 1] ];
  wTr_i = wTf*fTr_i;
  q_qs_plan(1:6,i) = [wTr_i(1:3,4); rotmat2rpy(wTr_i(1:3,1:3))];
end

qtraj = PPTrajectory(spline(t_qs_breaks,q_qs_plan));
comtraj = PPTrajectory(spline(t_qs_breaks,com_qs_plan(1:2,:)));
htraj = PPTrajectory(spline(t_qs_breaks,com_qs_plan(3,:)));
foot_support=PPTrajectory(zoh(t_qs_breaks,foot_support_qs));

Q = 1*eye(4);
R = 0.001*eye(2);
comgoal = com_qs_plan(1:2,end);
ltisys = LinearSystem([zeros(2),eye(2); zeros(2,4)],[zeros(2); eye(2)],[],[],[],[]);
[~,V] = tilqr(ltisys,Point(getStateFrame(ltisys),[comgoal;0*comgoal]),Point(getInputFrame(ltisys)),Q,R);

% compute TVLQR
options.tspan = linspace(com_qs_traj.tspan(1),com_qs_traj.tspan(2),10);
options.sqrtmethod = false;
x0traj = setOutputFrame([com_qs_traj(1:2);0;0],ltisys.getStateFrame);
u0traj = setOutputFrame(ConstantTrajectory([0;0]),ltisys.getInputFrame);
S = warning('off','Drake:TVLQR:NegativeS');  % i expect to have some zero eigenvalues, which numerically fluctuate below 0
warning(S);
[~,V] = tvlqr(ltisys,x0traj,u0traj,Q,R,V,options);

mu=0.5;
data = struct('S',V.S,'s1',V.s1,'s2',V.s2,...
  'support_times',support_times,'supports',{support_states},'comtraj',com_qs_traj,'qtraj',q_qs_traj,'mu',mu,...
  'link_constraints',[],'zmptraj',[],'qnom',[]);

pub =  WalkingPlanPublisher('QUASISTATIC_ROBOT_PLAN');
pub.publish(0,data);

end
