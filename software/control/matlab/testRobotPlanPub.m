function testRobotPlanPub

addpath(fullfile(pwd,'frames'));

pinned = true;

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

load('data/atlas_fp.mat');

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');

while true
  [x,~] = getNextMessage(state_frame,10);
  if (~isempty(x))
    x0=x;
    break
  end
end

nq = getNumDOF(r);
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);
com = getCOM(r,kinsol);

% create desired joint trajectory
cost = Point(r.getStateFrame,1);
% cost.base_x = 0;
% cost.base_y = 0;
% cost.base_z = 0;
% cost.base_roll = 1000;
% cost.base_pitch = 1000;
% cost.base_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));

  
rhand_body = r.findLink('r_hand');
lhand_body = r.findLink('l_hand');
rhand_pos = forwardKin(r,kinsol,rhand_body,[0;0;0],false);
lhand_pos = forwardKin(r,kinsol,lhand_body,[0;0;0],false);

pelvis_body = r.findLink('pelvis');
pelvis_pos = forwardKin(r,kinsol,pelvis_body,[0;0;0],true);  

q_nom(1:6) = pelvis_pos;
options.q_nom = q_nom;

rhand_goal = pelvis_pos(1:3) + [-0.4; -0.25; 0.35];
lhand_goal = pelvis_pos(1:3) + [0.4; -0.25; 0.35];

rfoot_body = r.findLink('r_foot');
lfoot_body = r.findLink('l_foot');
rfoot_pos = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot_pos = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

% time spacing of samples for IK
T = 5;
ts = 0:0.1:T;

rhand_traj = PPTrajectory(spline([0 T],[rhand_pos rhand_goal]));
lhand_traj = PPTrajectory(spline([0 T],[lhand_pos lhand_goal]));

for i=1:length(ts)
  t = ts(i);
  if (i>1)
    if pinned
      q(:,i) = inverseKin(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
        lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
        lhand_body,[0;0;0],lhand_traj.eval(t),pelvis_body,[0;0;0],pelvis_pos,options);
    else
      q(:,i) = approximateIK(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
        lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
        lhand_body,[0;0;0],lhand_traj.eval(t),options);
%     q(:,i) = inverseKin(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
%       lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
%       lhand_body,[0;0;0],lhand_traj.eval(t),options);
    end
  else
    q = q0;
  end
end

xtraj = [q; zeros(size(q))];
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
plan_pub.publish(ts,xtraj);

end
