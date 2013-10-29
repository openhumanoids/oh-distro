function [zmptraj, foottraj, support_times, supports] = planZMPTraj(biped, q0, X, options)

if ~isfield(options, 'ignore_terrain') options.ignore_terrain = false; end
if ~isfield(options, 'full_foot_pose_constraint') options.full_foot_pose_constraint = false; end
typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumDOF,1]);

debug = true;

is_right_foot = X(1).is_right_foot;

com0 = getCOM(biped,q0);
foot0 = feetPosition(biped, q0);
foot0.right(6) = foot0.left(6) + angleDiff(foot0.left(6), foot0.right(6));

steps.right = X([X.is_right_foot]);
steps.left = X(~[X.is_right_foot]);
steps.right(1).pos = foot0.right;
steps.left(1).pos = foot0.left;

for f = {'left', 'right'}
  foot = f{1};
  for k = 1:(length(steps.(foot))-1)
    steps.(foot)(k+1).pos(4:6) = steps.(foot)(k).pos(4:6) + angleDiff(steps.(foot)(k).pos(4:6), steps.(foot)(k+1).pos(4:6));
  end
end
for k = 1:length(steps.right)
  steps.right(k).center = biped.footOrig2Contact(steps.right(k).pos, 'center', 1);
end
for k = 1:length(steps.left)
  steps.left(k).center = biped.footOrig2Contact(steps.left(k).pos, 'center', 0);
end

function pos = feetCenter(rfootpos,lfootpos)
  rcen = biped.footOrig2Contact(rfootpos, 'center', 1);
  lcen = biped.footOrig2Contact(lfootpos, 'center', 0);
  pos = mean([rcen(1:3,:),lcen(1:3,:)],2);
end

supp0 = struct('right', steps.right(1).is_in_contact, 'left', steps.left(1).is_in_contact);
zmp0 = [];
if supp0.right
  zmp0(:,end+1) = steps.right(1).center(1:2);
end
if supp0.left
  zmp0(:,end+1) = steps.left(1).center(1:2);
end
zmp0 = mean(zmp0, 2);


step_knots = struct('t', 0, 'right', struct('orig', steps.right(1).pos), 'left', struct('orig', steps.left(1).pos));
zmp_knots = struct('t', 0, 'zmp', zmp0, 'supp', supp0);

if zmp_knots(1).supp.right && zmp_knots(1).supp.left
  zmp_knots(end+1) = zmp_knots(end);
  zmp_knots(end).t = 0.5;
  zmp_knots(end).zmp = com0(1:2);
  step_knots(end+1) = step_knots(end);
  step_knots(end).t = zmp_knots(end).t;
end

istep = struct('right', 1, 'left', 1);

while 1
  if is_right_foot
    sw_foot = 'right'; % moving (swing) foot
    st_foot = 'left'; % stance foot
  else
    sw_foot = 'left';
    st_foot = 'right';
  end
  options.step_speed = steps.(sw_foot)(istep.(sw_foot)).step_speed;
  options.step_height = steps.(sw_foot)(istep.(sw_foot)).step_height;

  [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped,...
                steps.(sw_foot)(istep.(sw_foot)).center,...
                steps.(sw_foot)(istep.(sw_foot)+1).center, options);

  t0 = step_knots(end).t;
  for j = 1:length(swing_ts)
    step_knots(end+1).t = swing_ts(j) + t0;
    step_knots(end).(sw_foot).orig = biped.footContact2Orig(swing_poses.center(:,j), 'center', is_right_foot);
    step_knots(end).(st_foot).orig = steps.(st_foot)(istep.(st_foot)).pos;
    
    if ~options.full_foot_pose_constraint && j >= 3 && j <= (length(swing_ts) - 4)
      % Release orientation constraints on the foot during the middle of the swing
      step_knots(end).(sw_foot).orig(4:5) = nan;
    end
  end

  step_duration = (swing_ts(end) - swing_ts(1));
  zmp1 = steps.(st_foot)(istep.(st_foot)).center(1:2);
  zmp2 = feetCenter(steps.(sw_foot)(istep.(sw_foot)+1).pos,...
                    steps.(st_foot)(istep.(st_foot)).pos);
  zmp2 = zmp2(1:2);
  supp1 = struct('right', ~is_right_foot, 'left', is_right_foot);
  supp2 = struct('right', 1, 'left', 1);
  zmp_knots(end+1) = struct('t', t0 + takeoff_time, 'zmp', zmp1, 'supp', supp1);
  zmp_knots(end+1) = struct('t', t0 + mean([takeoff_time, landing_time]), 'zmp', zmp1, 'supp', supp1);
  zmp_knots(end+1) = struct('t', t0 + landing_time, 'zmp', zmp1, 'supp', supp2);
  zmp_knots(end+1) = struct('t', t0 + landing_time + (2/3) * (step_duration - landing_time), 'zmp', zmp2, 'supp', supp2);
  zmp_knots(end+1) = struct('t', t0 + step_duration, 'zmp', zmp2, 'supp', supp2);

  istep.(sw_foot) = istep.(sw_foot) + 1;
  
  is_right_foot = ~is_right_foot;
  if istep.left == length(steps.left) && istep.right == length(steps.right)
    break
  end
end

% add a segment at the end to recover
t0 = step_knots(end).t;
step_knots(end+1) = step_knots(end);
step_knots(end).t = t0 + 1.5;
zmpf = feetCenter(step_knots(end).right.orig, step_knots(end).left.orig);
zmpf = zmpf(1:2);
zmp_knots(end+1) =  struct('t', step_knots(end).t, 'zmp', zmpf, 'supp', struct('right', 1, 'left', 1));

% Build trajectories
for f = {'right', 'left'}
  foot = f{1};
  footpos = [step_knots.(foot)];
  foottraj.(foot).orig = PPTrajectory(foh([step_knots.t], [footpos.orig]));
end
zmptraj = PPTrajectory(foh([zmp_knots.t], [zmp_knots.zmp]));

% create support body trajectory
rfoot_body_idx = biped.foot_bodies_idx(1);
lfoot_body_idx = biped.foot_bodies_idx(2);
support_times = [zmp_knots.t];
supps = [zmp_knots.supp];
foot_supports = [[supps.right] * rfoot_body_idx;
                 [supps.left] * lfoot_body_idx];
zmp_ts = [zmp_knots.t];
supports = cell(length(zmp_ts),1);
for i=1:length(zmp_ts)
  if all(foot_supports(:,i)~=0)
    cpts = {1:4,1:4};
  else
    cpts = {1:4};
  end
  supports{i} = SupportState(biped,foot_supports(:,i),cpts);
end

if debug
  ts = foottraj.right.orig.getBreaks();
  pts.right = foottraj.right.orig.eval(ts);
  pts.left = foottraj.left.orig.eval(ts);
  plot_lcm_points(pts.right', repmat([224/255, 116/255, 27/255], size(pts.right, 2), 1), 30, 'Right Foot Trajectory', 2, 1);
  plot_lcm_points(pts.left', repmat([27/255, 148/255, 224/255], size(pts.left, 2), 1), 31, 'Left Foot Trajectory', 2, 1);
  tt = zmp_ts;
  zmppoints = zeros(3,length(tt));
  zmppoints(1:2,:) = zmptraj.eval(tt);
  zmppoints(3,:) = getTerrainHeight(biped,zmppoints(1:2,:));
  plot_lcm_points(zmppoints',zeros(length(tt),3),67676,'ZMP location',2,true);
end

end


