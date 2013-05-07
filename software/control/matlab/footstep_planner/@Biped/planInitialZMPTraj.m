function [zmptraj, foottraj, supporttraj] = planInitialZMPTraj(biped, q0, X)

debug = true;

Xpos = [X.pos];
% step_times = [X.time];
time_ndx = 2;
Xright = Xpos(:, [X.is_right_foot] == 1);
Xleft = Xpos(:, [X.is_right_foot] == 0);
bRightStep = X(1).is_right_foot;

typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumDOF,1]);

step_locations = struct('right', Xright(1:6,:), 'left', Xleft(1:6,:));

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
footpos = struct('right', struct(), 'left', struct());
footpos.right.orig = [foot0.right, Xright(1:6,1)];
footpos.left.orig = [foot0.left, Xleft(1:6,1)];

% foottraj.right.orig = PPTrajectory(foh(ts, footpos.right.orig));
% foottraj.left.orig = PPTrajectory(foh(ts, footpos.left.orig));

zmp = [com0(1:3), feetCenter(footpos.right.orig(:,2), footpos.left.orig(:,2))];
zmp = zmp(1:2,:);

istep = struct('right', 1, 'left', 1);

footsupport.right = [1 1];
footsupport.left = [1 1];


while 1
  % step.left.orig = repmat(footpos.left.orig(:,end), 1, 5);
  % step.right.orig = repmat(footpos.right.orig(:,end), 1, 5);
% %   step_time = step_times(time_ndx + 2) - step_times(time_ndx);
%   time_ndx = time_ndx + 2;
  
% %   tstep = ts(end) + [.3, .45, .6, .9, 1] * step_time;
  if bRightStep
    m_foot = 'right'; % moving (swing) foot
    s_foot = 'left'; % stance foot
  else
    m_foot = 'left';
    s_foot = 'right';
  end
  
  % step.(m_foot).orig = [step_locations.(m_foot)(1:6, istep.(m_foot)),...
  %                       step_locations.(m_foot)(1:6, istep.(m_foot)+1),...
  %                repmat(step_locations.(m_foot)(1:6, istep.(m_foot)+2), 1, 3)];
  % step.(m_foot).orig(3,:) = step.(m_foot).orig(3,:) + [0, 0, 0.005, 0, 0];
  
  is_right_foot = strcmp(m_foot, 'right')
  [step_traj.(m_foot).center, takeoff_time, landing_time] = planSwing(biped,...
    biped.footOrig2Contact(step_locations.(m_foot)(1:6, istep.(m_foot)), 'center', is_right_foot),...
    biped.footOrig2Contact(step_locations.(m_foot)(1:6, istep.(m_foot)+2), 'center', is_right_foot),...
    biped.footOrig2Contact(step_locations.(m_foot)(1:6, istep.(m_foot)+1), 'center', is_right_foot));
  tstep = ts(end) + step_traj.(m_foot).center.getBreaks();
  step.(m_foot).orig = biped.footContact2Orig(...
    step_traj.(m_foot).center.eval(step_traj.(m_foot).center.getBreaks()),...
    'center', strcmp(m_foot, 'right'));
  step.(s_foot).orig = repmat(step_locations.(s_foot)(1:6, istep.(s_foot)), 1, length(tstep));

  % step_traj.(m_foot).orig = PPTrajectory(foh(traj_ts, traj_pts));

  % step_traj.(s_foot).orig = PPTrajectory(foh(step_traj.(m_foot).orig.getTimeSpan(), ...
  %   repmat(step_locations.(s_foot)(1:6, istep.(s_foot)), 1, 2)));
  
  % tspan = step_traj.(m_foot).orig.getTimeSpan();
  % tstep = ts(end) + [.1, .5, .85, .95, 1] * (tspan(end) - tspan(1));
  % zmp_tstep = ts(end) + [.1, .5, .85, .95, 1] * (tstep(end) - tstep(1));
  step_duration = (tstep(end) - tstep(1));
  zmp_tstep = ts(end) + [takeoff_time, mean([takeoff_time, landing_time]),...
                         landing_time, landing_time + (2/3) * (step_duration - landing_time),...
                         step_duration];

  % Shift the ZMP by 2cm closer to the center of the feet
  foot_center = biped.footOrig2Contact(step_locations.(s_foot)(1:6,istep.(s_foot)), 'center', strcmp(s_foot, 'right'));
%   foot_center = biped.footOrig2Contact(step.(s_foot).orig(:,1), 'center', strcmp(s_foot, 'right'));
  step_center = biped.footCenter2StepCenter(foot_center, strcmp(s_foot, 'right'));
  zmp_shift = [(step_center(1:2) - foot_center(1:2)); 0];
  zmp_shift = zmp_shift ./ sqrt(sum(zmp_shift.^2)) * 0; % shift ZMP toward instep 
  s_foot_center = biped.footOrig2Contact(step.(s_foot).orig(:,1), 'center', strcmp(s_foot, 'right'));
  stepzmp = [repmat(s_foot_center(1:3)+zmp_shift,1,3)...
             repmat(feetCenter(step.(m_foot).orig(:,end), step.(s_foot).orig(:,end)), 1, 2)];

  
  footsupport.(m_foot) = [footsupport.(m_foot), [[0 0 1 1 1] + [0 0, repmat(step_locations.(m_foot)(3, istep.(m_foot)+2), 1, 3)]]] ; 
  footsupport.(s_foot) = [footsupport.(s_foot), [[1 1 1 1 1] + [repmat(step_locations.(s_foot)(3, istep.(s_foot)), 1, 5)]]]; 

  istep.(m_foot) = istep.(m_foot) + 2;
  
  for f = {'right', 'left'}
    foot = f{1};
%     tspan = foottraj.(foot).orig.getTimeSpan();
%     step_traj.(foot).orig = step_traj.(foot).orig.shiftTime(tspan(end));
    % foottraj.(foot).orig = foottraj.(foot).orig.append(step_traj.(foot).orig);
    footpos.(foot).orig = [footpos.(foot).orig, step.(foot).orig];
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
  foottraj.(foot).orig = PPTrajectory(foh(ts, footpos.(foot).orig));
  footsupport.(foot) = [footsupport.(foot), 1 + footsupport.(foot)(end)];
end

% create ZMP trajectory
p = feetCenter(footpos.right.orig(:,end),footpos.left.orig(:,end));
zmp = [zmp,p(1:2)];
zmptraj = PPTrajectory(foh(zmp_ts,zmp));

% create support body trajectory
supporttraj = repmat(0*zmp_ts,length(biped.getLinkNames),1);
supporttraj(strcmp(biped.r_foot_name,biped.getLinkNames),:) = footsupport.right;
supporttraj(strcmp(biped.l_foot_name,biped.getLinkNames),:) = footsupport.left;
supporttraj = setOutputFrame(PPTrajectory(zoh(zmp_ts,supporttraj)),AtlasBody(biped));

if debug
  tt = 0:0.02:zmp_ts(end);
  zmppoints = ones(3,length(tt));
  for i=1:length(tt)
    zmppoints(1:2,i) = zmptraj.eval(tt(i));
  end
  zmppoints(3,:) = getTerrainHeight(biped,zmppoints(1:2,:));
  plot_lcm_points(zmppoints',zeros(length(tt),3),67676,'ZMP location',1,true);
  % biped.plot_step_clearance_lcm(footpos);
end

end


