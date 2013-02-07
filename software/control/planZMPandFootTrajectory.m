function [zmptraj,lfoottraj,rfoottraj,step_times] = planZMPandFootTrajectory(r,q0, Xright, Xleft,step_time)

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[r.getNumDOF,1]);
sizecheck(step_time,1);

kinsol = doKinematics(r,q0);
rfoot_body = findLink(r,'r_foot');
lfoot_body = findLink(r,'l_foot');

com0 = getCOM(r,q0);
rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);

gc = r.contactPositions(q0);

% compute desired COM projection
% assumes minimal contact model for now
k = convhull(gc(1:2,1:4)');
lfootcen0 = [mean(gc(1:2,k),2);0];
k = convhull(gc(1:2,5:8)');
rfootcen0 = [mean(gc(1:2,4+k),2);0];
roffset = rfootcen0 - rfoot0(1:3);
loffset = lfootcen0 - lfoot0(1:3);

function pos = rfootCenter(rfootpos)
  yaw = rfootpos(6);
  offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * roffset(1:2);
  pos = rfootpos(1:2)+offset;
end    
  
function pos = lfootCenter(lfootpos)
  yaw = lfootpos(6);
  offset = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)] * loffset(1:2);
  pos = lfootpos(1:2)+offset;
end

function pos = feetCenter(rfootpos,lfootpos)
  rcen = rfootCenter(rfootpos);
  lcen = lfootCenter(lfootpos);
%   rcen = rfootpos(1:2)+roffset(1:2);
%   lcen = lfootpos(1:2)+loffset(1:2);
  pos = mean([rcen,lcen],2);
end


ts = [0, .5];
step_times = [0];
rfootpos = [rfoot0, rfoot0];
lfootpos = [lfoot0, lfoot0];
zmp = [com0(1:2), feetCenter(rfootpos(:,2), lfootpos(:,2))];


% p0 = feetCenter(rfoot0, lfoot0);
% start_pos = [p0; 0; 0; 0; atan2(lfoot0(2) - rfoot0(2), lfoot0(1) - rfoot0(1)) - pi/2];
% step_width = sqrt(sum((lfoot0(1:2) - rfoot0(1:2)).^2))
% poses = [start_pos, goal_poses];
% traj = turnGoTraj(poses);
% [Xright, Xleft, X, lambda] = constrainedFootsteps(traj, step_length, step_width, pi/16);
% figure(21)
% plotFootstepPlan(traj, lambda, step_width)
% drawnow

bRightStep = true;
istep_r = 1;
istep_l = 1;

while 1
  lf = repmat(lfootpos(:,end), 1, 4);
  rf = repmat(rfootpos(:,end), 1, 4);
  tstep = ts(end) + [.3, .45, .6, .9, 1] * step_time;
  if bRightStep
    rf(1,:) = rf(1,:) + [0, (Xright(1, istep_r+1) - Xright(1, istep_r)) / 2, ...
      (Xright(1, istep_r+1) - Xright(1, istep_r)), (Xright(1, istep_r+1) - Xright(1, istep_r))];
    rf(2,:) = rf(2,:) + [0, (Xright(2, istep_r+1) - Xright(2, istep_r)) / 2, ...
      (Xright(2, istep_r+1) - Xright(2, istep_r)), (Xright(2, istep_r+1) - Xright(2, istep_r))];
    rf(3,:) = rf(3,:) + [0, 0.05, 0, 0];
    rf(6,:) = rf(6,:) + [0, (Xright(6, istep_r+1) - Xright(6, istep_r)) / 2,...
      (Xright(6, istep_r+1) - Xright(6, istep_r)), (Xright(6, istep_r+1) - Xright(6, istep_r))];
    stepzmp = [repmat(lfootCenter(lf(:,1)),1,3),feetCenter(rf(:,end),lf(:,end))];
    istep_r = istep_r + 1;
  else
    lf(1,:) = lf(1,:) + [0, (Xleft(1, istep_l+1) - Xleft(1, istep_l)) / 2, ...
      (Xleft(1, istep_l+1) - Xleft(1, istep_l)), (Xleft(1, istep_l+1) - Xleft(1, istep_l))];
    lf(2,:) = lf(2,:) + [0, (Xleft(2, istep_l+1) - Xleft(2, istep_l)) / 2, ...
      (Xleft(2, istep_l+1) - Xleft(2, istep_l)), (Xleft(2, istep_l+1) - Xleft(2, istep_l))];
    lf(3,:) = lf(3,:) + [0, 0.05, 0, 0];
    lf(6,:) = lf(6,:) + [0, (Xleft(6, istep_l+1) - Xleft(6, istep_l)) / 2,...
      (Xleft(6, istep_l+1) - Xleft(6, istep_l)), (Xleft(6, istep_l+1) - Xleft(6, istep_l))];
    stepzmp = [repmat(rfootCenter(rf(:,1)),1,3),feetCenter(rf(:,end),lf(:,end))];
    istep_l = istep_l + 1;
  end
  rfootpos = [rfootpos, rf, rf(:,end)];
  lfootpos = [lfootpos, lf, lf(:,end)];
  zmp = [zmp, stepzmp, stepzmp(:,end)];
  ts = [ts, tstep];
  step_times = [step_times, ts(end)];
  bRightStep = ~bRightStep;
  if istep_l == length(Xleft(1,:)) && istep_r == length(Xright(1,:))
    break
  end
end

zmptraj = PPTrajectory(foh(ts,zmp));
lfoottraj = PPTrajectory(foh(ts,lfootpos));
rfoottraj = PPTrajectory(foh(ts,rfootpos));

end
