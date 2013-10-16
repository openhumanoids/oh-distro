function testRobotPlanPub
% NOTEST

addpath(fullfile(pwd,'frames'));

pinned = true;

options.floating = true;
options.dt = 0.001;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));

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
cost.back_bky = 100;
cost.back_bkx = 100;
cost = double(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(diag(cost(1:r.getNumDOF)));

  
rhand_body = r.findLinkInd('r_hand');
lhand_body = r.findLinkInd('l_hand');
rhand_pos = forwardKin(r,kinsol,rhand_body,[0;0;0],false);
lhand_pos = forwardKin(r,kinsol,lhand_body,[0;0;0],false);

pelvis_body = r.findLinkINd('pelvis');
pelvis_pos = forwardKin(r,kinsol,pelvis_body,[0;0;0],true);  

q_nom(1:6) = pelvis_pos;

rhand_goal = pelvis_pos(1:3) + [-0.4; -0.25; 0.35];
lhand_goal = pelvis_pos(1:3) + [0.4; -0.25; 0.35];

rfoot_body = r.findLinkInd('r_foot');
lfoot_body = r.findLinkInd('l_foot');
rfoot_pos = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot_pos = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

% time spacing of samples for IK
T = 5;
ts = 0:0.1:T;

rhand_traj = PPTrajectory(spline([0 T],[rhand_pos rhand_goal]));
lhand_traj = PPTrajectory(spline([0 T],[lhand_pos lhand_goal]));

kc_com = WorldCoMConstraint(r,1,[com(1:2);nan],[com(1:2);nan]);
kc_rfoot1 = WorldPositionConstraint(r,rfoot_body,[0;0;0],rfoot_pos(1:3),rfoot_pos(1:3));
kc_rfoot2 = WorldQuatConstraint(r,rfoot_body,rpy2quat(rfoot_pos(4:end)),0);
kc_lfoot1 = WorldPositionConstraint(r,lfoot_body,[0;0;0],lfoot_pos(1:3),lfoot_pos(1:3));
kc_lfoot2 = WorldQuatConstraint(r,lfoot_body,rpy2quat(lfoot_pos(4:end)),0);
kc_pelvis1 = WorldPositionConstraint(r,pelvis_body,[0;0;0],pelvis_pos(1:3),pelvis_pos(1:3));
kc_pelvis2 = WorldQuatConstraint(r,pelvis_body,rpy2quat(pelvis_pos(4:end)),0);

for i=1:length(ts)
  t = ts(i);
  if (i>1)
    if pinned
      kc_rhand = WorldPositionConstraint(r,rhand_body,[0;0;0],rhand_traj.eval(t));
      kc_lhand = WorldPositionConstraint(r,lhand_body,[0;0;0],lhand_traj.eval(t));
      q(:,i) = inverseKin(r,q_nom,q_nom,kc_com,kc_rfoot1,kc_rfoot2,kc_lfoot1,kc_lfoot2,...
        kc_pelvis1,kc_pelvis2,kc_rhand,kc_lhand,ikoptions);
%       q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
%         lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
%         lhand_body,[0;0;0],lhand_traj.eval(t),pelvis_body,[0;0;0],pelvis_pos,options);
    else
      [q(:,i),info] = approximateIK(r,q_nom,q_nom,kc_com,kc_rfoot1,kc_rfoot2,kc_lfoot1,kc_lfoot2,...
        kc_pelvis1,kc_pelvis2,kc_rhand,kc_lhand,ikoptions);
%       [q(:,i),info] = approximateIK(r,q(:,i-1),0,[com(1:2);nan],rfoot_body,[0;0;0],rfoot_pos, ...
%         lfoot_body,[0;0;0],lfoot_pos,rhand_body,[0;0;0],rhand_traj.eval(t), ...
%         lhand_body,[0;0;0],lhand_traj.eval(t),options);
      if info, error('approximate IK reported an error'); end
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
plan_pub = drc.control.RobotPlanPublisher(joint_names,true,'CANDIDATE_ROBOT_PLAN');
plan_pub.publish(ts,xtraj);

end
