function testRobotPlanPub

addpath(fullfile(pwd,'frames'));

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

% set initial state to fixed point
load('data/atlas_fp.mat');
xstar(1:6) = [0 0 2.0 0 0 0]'; % pinned position
r = r.setInitialState(xstar);

nq = getNumDOF(r);
q0 = xstar(1:nq);
kinsol = doKinematics(r,q0);

cost = Point(r.getStateFrame,1);
cost.base_x = 100;
cost.base_y = 100;
cost.base_z = 100;
cost.base_roll = 100;
cost.base_pitch = 100;
cost.base_yaw = 0;
cost.back_mby = 10;
cost.back_ubx = 10;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q0;
  
rhand_body = r.findLink('r_hand');
lhand_body = r.findLink('l_hand');
pelvis_body = r.findLink('pelvis');
rhand_pos = forwardKin(r,kinsol,rhand_body,[0;0;0],false);
lhand_pos = forwardKin(r,kinsol,lhand_body,[0;0;0],false);
rhand_goal = [0.2; -0.15; 2.65];
lhand_goal = [0.4; 0.25; 2.0];

% time spacing of samples for IK
T = 5;
ts = 0:0.1:T;

rhand_traj = PPTrajectory(spline([0 T],[rhand_pos rhand_goal]));
lhand_traj = PPTrajectory(spline([0 T],[lhand_pos lhand_goal]));

for i=1:length(ts)
  t = ts(i);
  if (i>1)
    q(:,i) = approximateIK(r,q(:,i-1),pelvis_body,[0;0;0],[0;0;2.0;0;0;0], ...
      rhand_body,[0;0;0],rhand_traj.eval(t), lhand_body,[0;0;0],lhand_traj.eval(t),options);
  else
    q = q0;
  end
end

xtraj = [q; zeros(size(q))];
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
plan_pub = RobotPlanPublisher('atlas',joint_names,true,'COMMITTED_ROBOT_PLAN');
plan_pub.publish(ts,xtraj);

end
