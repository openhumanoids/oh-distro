function [zmptraj, foottraj, support_times, supports] = planInitialZMPTraj(biped, q0, X, options)

debug = true;

if ~isfield(options, 'ignore_terrain') options.ignore_terrain = false; end

Xpos = [X.pos];
% time_ndx = 2;
step_locations.right = Xpos(:, [X.is_right_foot] == 1);
step_locations.left = Xpos(:, [X.is_right_foot] == 0);

bRightStep = X(1).is_right_foot;

typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumDOF,1]);


kinsol = doKinematics(biped,q0);

foot_body = struct('right', findLink(biped, biped.r_foot_name),...
  'left', findLink(biped, biped.l_foot_name));

com0 = getCOM(biped,q0);
foot0 = struct('right', forwardKin(biped,kinsol,foot_body.right,[0;0;0],true),...
  'left', forwardKin(biped,kinsol,foot_body.left,[0;0;0],true));

function pos = feetCenter(rfootpos,lfootpos)
  rcen = biped.footOrig2Contact(rfootpos, 'center', 1);
  lcen = biped.footOrig2Contact(lfootpos, 'center', 0);
  pos = mean([rcen(1:3,:),lcen(1:3,:)],2);
end

ts = [0, .5];
zmp_ts = ts;

% foot0.right(6) = foot0.left(6) + angleDiff(foot0.left(6), foot0.right(6));
unwrapped = unwrap([foot0.left(6), foot0.right(6)]);
foot0.right(6) = unwrapped(2);
for j = 4:6
  for f = {'left', 'right'}
    foot = f{1};
    step_locations.(foot)(j, 1) = foot0.(foot)(j) + angleDiff(foot0.(foot)(j), step_locations.(foot)(j,1));
    for k = 1:(size(step_locations.(foot), 2) - 1)
      step_locations.(foot)(j, k+1) = step_locations.(foot)(j, k) + angleDiff(step_locations.(foot)(j,k), step_locations.(foot)(j, k+1));
    end
  end
end
footpos = struct('right', struct(), 'left', struct());
footpos.right.orig = [foot0.right, step_locations.right(1:6,1)];
footpos.left.orig = [foot0.left, step_locations.left(1:6,1)];
footpos.right.toe.min = nan*ones(3,2);
footpos.right.toe.max = nan*ones(3,2);
footpos.right.heel.min = nan*ones(3,2);
footpos.right.heel.max = nan*ones(3,2);
footpos.left.toe.min = nan*ones(3,2);
footpos.left.toe.max = nan*ones(3,2);
footpos.left.heel.min = nan*ones(3,2);
footpos.left.heel.max = nan*ones(3,2);

zmp = [com0(1:3), feetCenter(footpos.right.orig(:,2), footpos.left.orig(:,2))];
zmp = zmp(1:2,:);

istep = struct('right', 1, 'left', 1);

footsupport.right = [1 1];
footsupport.left = [1 1];


while 1
  if bRightStep
    m_foot = 'right'; % moving (swing) foot
    s_foot = 'left'; % stance foot
  else
    m_foot = 'left';
    s_foot = 'right';
  end
  
  is_right_foot = strcmp(m_foot, 'right');
  options.foot_speed = X(istep.right + istep.left).step_speed;
  options.step_height = X(istep.right + istep.left).step_height;
  [swing_ts, swing_poses, takeoff_time, landing_time] = planSwing(biped,...
    biped.footOrig2Contact(step_locations.(m_foot)(1:6, istep.(m_foot)), 'center', is_right_foot),...
    biped.footOrig2Contact(step_locations.(m_foot)(1:6, istep.(m_foot)+1), 'center', is_right_foot), options);
  tstep = ts(end) + swing_ts;
  step.(m_foot) = swing_poses;
  step.(m_foot).orig = biped.footContact2Orig(swing_poses.center,'center',strcmp(m_foot, 'right'));
  % step.(m_foot).toe = toe_heights;
  stance_pos = step_locations.(s_foot)(1:6, istep.(s_foot));
  % stance_toe = biped.footOrig2Contact(stance_pos,'toe', strcmp(s_foot, 'right'));
  step.(s_foot).orig = repmat(stance_pos, 1, length(tstep));
  step.(s_foot).toe.min = repmat(nan, 3, length(tstep));
  step.(s_foot).toe.max = repmat(nan, 3, length(tstep)); 
  step.(s_foot).heel.min = repmat(nan, 3, length(tstep)); 
  step.(s_foot).heel.max = repmat(nan, 3, length(tstep)); 

  % Release orientation constraints on the foot during the middle of the swing
  step.(m_foot).orig(4:5,3:end-4) = nan;

  step_duration = (tstep(end) - tstep(1));
  zmp_tstep = ts(end) + [takeoff_time, mean([takeoff_time, landing_time]),...
                         landing_time, landing_time + (2/3) * (step_duration - landing_time),...
                         step_duration];

  % Shift the ZMP by 2cm closer to the center of the feet
  foot_center = biped.footOrig2Contact(step_locations.(s_foot)(1:6,istep.(s_foot)), 'center', strcmp(s_foot, 'right'));
  step_center = biped.footCenter2StepCenter(foot_center, strcmp(s_foot, 'right'));
  zmp_shift = [(step_center(1:2) - foot_center(1:2)); 0];
  zmp_shift = zmp_shift ./ sqrt(sum(zmp_shift.^2)) * 0.01; % shift ZMP toward instep 
  s_foot_center = biped.footOrig2Contact(step.(s_foot).orig(:,1), 'center', strcmp(s_foot, 'right'));
  stepzmp = [repmat(s_foot_center(1:3)+zmp_shift,1,3)...
             repmat(feetCenter(step.(m_foot).orig(:,end), step.(s_foot).orig(:,end)), 1, 2)];
  
  footsupport.(m_foot) = [footsupport.(m_foot), [0 0 1 1 1]] ; 
  footsupport.(s_foot) = [footsupport.(s_foot), [1 1 1 1 1]]; 

  istep.(m_foot) = istep.(m_foot) + 1;
  
  for f = {'right', 'left'}
    foot = f{1};
    footpos.(foot).orig = [footpos.(foot).orig, step.(foot).orig];
    footpos.(foot).toe.min = [footpos.(foot).toe.min, step.(foot).toe.min];
    footpos.(foot).toe.max = [footpos.(foot).toe.max, step.(foot).toe.max];
    footpos.(foot).heel.min = [footpos.(foot).heel.min, step.(foot).heel.min];
    footpos.(foot).heel.max = [footpos.(foot).heel.max, step.(foot).heel.max];
  end
  zmp = [zmp, stepzmp(1:2,:)];
  ts = [ts, tstep];
  zmp_ts = [zmp_ts, zmp_tstep];
  bRightStep = ~bRightStep;
  if istep.left == length(step_locations.left(1,:)) && istep.right == length(step_locations.right(1,:))
    break
  end
end

% add a segment at the end to recover
ts = [ts, ts(end)+1.5];
zmp_ts = [zmp_ts, zmp_ts(end)+1.5];

for f = {'right', 'left'}
  foot = f{1};
  
  % add a segment at the end to recover
  footpos.(foot).orig = [footpos.(foot).orig footpos.(foot).orig(:,end)];
  footpos.(foot).toe.min = [footpos.(foot).toe.min footpos.(foot).toe.min(:,end)];
  footpos.(foot).toe.max = [footpos.(foot).toe.max footpos.(foot).toe.max(:,end)];
  footpos.(foot).heel.min = [footpos.(foot).heel.min footpos.(foot).heel.min(:,end)];
  footpos.(foot).heel.max = [footpos.(foot).heel.max footpos.(foot).heel.max(:,end)];

  % build trajectories
  foottraj.(foot).orig = PPTrajectory(foh(ts, footpos.(foot).orig));
  for g = {'toe', 'heel'}
    grp = g{1};
    for l = {'min', 'max'}
      lim = l{1};
      if all(all(isnan(footpos.(foot).(grp).(lim))))
        foottraj.(foot).(grp).(lim) = ConstantTrajectory(repmat(nan, size(footpos.(foot).(grp).(lim), 1), 1));
      else
        foottraj.(foot).(grp).(lim) = PPTrajectory(foh(ts, footpos.(foot).(grp).(lim)));
      end
    end
  end
  footsupport.(foot) = [footsupport.(foot), 1];
end

% create ZMP trajectory
p = feetCenter(footpos.right.orig(:,end),footpos.left.orig(:,end));
zmp = [zmp,p(1:2)];
zmptraj = PPTrajectory(foh(zmp_ts,zmp));

% create support body trajectory
rfoot_body_idx = findLinkInd(biped,biped.r_foot_name);
lfoot_body_idx = findLinkInd(biped,biped.l_foot_name);
support_times = zmp_ts;
foot_supports = [footsupport.right * rfoot_body_idx; footsupport.left * lfoot_body_idx];
supports = cell(length(zmp_ts),1);
for i=1:length(zmp_ts)
  supports{i} = SupportState(biped,foot_supports(:,i));
end

if debug
  tt = zmp_ts;
  zmppoints = zeros(3,length(tt));
  zmppoints(1:2,:) = zmptraj.eval(tt);
  zmppoints(3,:) = getTerrainHeight(biped,zmppoints(1:2,:));
  plot_lcm_points(zmppoints',zeros(length(tt),3),67676,'ZMP location',2,true);
  % biped.plot_step_clearance_lcm(footpos);
end

end


