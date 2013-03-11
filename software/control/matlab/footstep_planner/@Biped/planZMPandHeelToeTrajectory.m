function [zmptraj,foottraj,contact_ref,step_times,supporttraj] = planZMPandHeelToeTrajectory(biped,q0, Xright, Xleft,step_time,options)
if nargin < 6
  options = struct();
end
if ~isfield(options, 'flat_foot')
  options.flat_foot = true;
end

typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumDOF,1]);
sizecheck(step_time,1);

step_locations = struct('right', Xright, 'left', Xleft);

kinsol = doKinematics(biped,q0);

foot_body = struct('right', findLink(biped, biped.r_foot_name),...
  'left', findLink(biped, biped.l_foot_name));

com0 = getCOM(biped,q0);
foot0 = struct('right', forwardKin(biped,kinsol,foot_body.right,[0;0;0],true),...
  'left', forwardKin(biped,kinsol,foot_body.left,[0;0;0],true));

if true%~options.flat_foot
  group_pts = struct('left', struct(), 'right', struct());
  for g = {'toe', 'heel'}
    grp = g{1};
    for f = {'right', 'left'}
      foot = f{1};
      group_pts.(foot).(grp) = foot_body.(foot).contact_pts(:,...
        foot_body.(foot).collision_group{...
            cellfun(@(x) strcmp(x, grp), ...
             foot_body.(foot).collision_group_name)});
    end
  end
end

foot_cen0 = struct();
contact_ref = struct('right', struct(), 'left', struct());
offset = struct();
for f = {'right', 'left'}
  foot = f{1};
  gc = [group_pts.(foot).toe, group_pts.(foot).heel];
  gc = forwardKin(biped, kinsol, foot_body.(foot), gc, true);
  k = convhull(gc(1:2,:)');
  foot_cen0.(foot) = mean(gc(1:3, k),2);
  offset.(foot).center = foot_cen0.(foot) - foot0.(foot)(1:3);
  
  if ~options.flat_foot
    for g = {'toe', 'heel'}
      grp = g{1};
      gc = [group_pts.(foot).(grp)];
      contact_ref.(foot).(grp) = mean(gc, 2);
      contact_pos = forwardKin(biped, kinsol, foot_body.(foot), contact_ref.(foot).(grp), true);
      offset.(foot).(grp) = contact_pos(1:3) - foot0.(foot)(1:3);
    end
  end
end

function fpos = footPoint(foot, grp, pos)
  % Return the position in the lab frame of the center of the contact group
  % [grp] with [foot] at position [pos]. For example, 
  % footPoint('right','center',[0;0;0;0;0;0]) gives the position of the
  % center of the right foot contact when that foot's origin is at O.
  yaw = pos(6,:);
  for j = 1:length(yaw)
    offs = [cos(yaw(j)), -sin(yaw(j)), 0; sin(yaw(j)), cos(yaw(j)), 0; 0, 0, 1]...
            * offset.(foot).(grp)(1:3);
    fpos(:,j) = pos(1:3,j) + offs;
  end
end

function pos = feetCenter(rfootpos,lfootpos)
  rcen = footPoint('right', 'center', rfootpos);
  lcen = footPoint('left', 'center', lfootpos);
  pos = mean([rcen(1:3,:),lcen(1:3,:)],2);
end



ts = [0, .5];
step_times = [0];
footpos = struct('right', struct(), 'left', struct());
footpos.right.orig = [foot0.right, foot0.right];
footpos.left.orig = [foot0.left, foot0.left];

if ~options.flat_foot
  for f = {'right', 'left'}
    foot = f{1};
    for g = {'toe', 'heel'}
      grp = g{1};
      for b = {'ub', 'lb'}
        bound = b{1};
        footpos.(foot).(grp).(bound) = footPoint(foot, grp, footpos.(foot).orig);
      end
    end
  end
end

zmp = [com0(1:3), feetCenter(footpos.right.orig(:,2), footpos.left.orig(:,2))];
zmp = zmp(1:2,:);

bRightStep = size(Xright,2) >= size(Xleft,2);
istep = struct('right', 1, 'left', 1);

footsupport.right = [1 1];
footsupport.left = [1 1];


while 1
  step.left.orig = repmat(footpos.left.orig(:,end), 1, 5);
  step.right.orig = repmat(footpos.right.orig(:,end), 1, 5);
  
%   tstep = ts(end) + [.3, .45, .6, .9, 1] * step_time;
  tstep = ts(end) + [.1, .5, .925, .975, 1] * step_time;
  if bRightStep
    m_foot = 'right';
    s_foot = 'left';
  else
    m_foot = 'left';
    s_foot = 'right';
  end
  
  step.(m_foot).orig = interp1([0; 1], [step_locations.(m_foot)(:, istep.(m_foot)), step_locations.(m_foot)(:, istep.(m_foot)+1)]', [0, .5, 1, 1, 1]')';
  step.(m_foot).orig(3,:) = step.(m_foot).orig(3,:) + [0, 0.05, 0, 0, 0];
  for i = 1:length(step.(m_foot).orig(1,:))
    R = makehgtform('zrotate', step.(m_foot).orig(6,i));
    offs = R * [offset.(m_foot).center; 1];
    step.(m_foot).orig(1:3, i) = step.(m_foot).orig(1:3, i) - offs(1:3);
  end
  
  if ~options.flat_foot
    stepzmp = [footPoint(s_foot, 'center', step.(s_foot).orig(:,1)),...
               footPoint(s_foot, 'toe', step.(s_foot).orig(:,1)),...
               repmat(mean([footPoint(s_foot, 'toe', step.(s_foot).orig(:,1)),...
                            footPoint(m_foot, 'heel', step.(m_foot).orig(:,1))], 2), 1, 3)];
    step.(m_foot).toe.lb = footPoint(m_foot, 'toe', step.(m_foot).orig(:,1:5)) ...
                           - [zeros(3, 2), [0.1;0;0], zeros(3,2)];
    step.(m_foot).toe.ub = footPoint(m_foot, 'toe', step.(m_foot).orig(:,1:5)) ...
                           + [zeros(3, 2), [0;0;0.1], zeros(3,2)];
    step.(m_foot).heel.lb = [footPoint(m_foot, 'heel', step.(m_foot).orig(:,1:5))];
    step.(m_foot).heel.ub = footPoint(m_foot, 'heel', step.(m_foot).orig(:,:))...
                            + [[0.1;0;0.1], [0.1;0;0.1], zeros(3,3)];
   
    for g = {'toe', 'heel'}
      grp = g{1};
      for b = {'lb', 'ub'}
        bound = b{1};
        step.(s_foot).(grp).(bound) = footPoint(s_foot, grp, step.(s_foot).orig(:,1:5));
        footpos.(foot).(grp).(bound) = [footpos.(foot).(grp).(bound), step.(foot).(grp).(bound)];
      end
    end
  else
    stepzmp = [repmat(footPoint(s_foot, 'center', step.(s_foot).orig(:,1)),1,3),...
               repmat(feetCenter(step.(m_foot).orig(:,end), step.(s_foot).orig(:,end)), 1, 2)];
    % step.(m_foot).toe.lb = footPoint(m_foot, 'toe', step.(m_foot).orig(:,:))-repmat(0.01, 3,5);
    % step.(m_foot).toe.ub = footPoint(m_foot, 'toe', step.(m_foot).orig(:,:))+repmat(0.01, 3,5);
    % step.(m_foot).heel.lb = footPoint(m_foot, 'heel', step.(m_foot).orig(:,:))-repmat(0.01, 3,5);
    % step.(m_foot).heel.ub = footPoint(m_foot, 'heel', step.(m_foot).orig(:,:))+repmat(0.01, 3,5);
  end
    
  istep.(m_foot) = istep.(m_foot) + 1;
  
  footsupport.(m_foot) = [footsupport.(m_foot) 0 0 .5 1 1]; 
  footsupport.(s_foot) = [footsupport.(s_foot) 1 1 1 1 1]; 
  
  for f = {'right', 'left'}
    foot = f{1};
    footpos.(foot).orig = [footpos.(foot).orig, step.(foot).orig];
  end
  zmp = [zmp, stepzmp(1:2,:)];
  ts = [ts, tstep];
  step_times = [step_times, tstep(2), tstep(end)];
  bRightStep = ~bRightStep;
  if istep.left == length(step_locations.left(1,:)) && istep.right == length(step_locations.right(1,:))
    break
  end
end


% add a segment at the end to recover
ts = [ts, ts(end)+1];

for f = {'right', 'left'}
  foot = f{1};
  % add a segment at the end to recover
  footpos.(foot).orig = [footpos.(foot).orig footpos.(foot).orig(:,end)];
  foottraj.(foot).orig = PPTrajectory(foh(ts, footpos.(foot).orig));
  if ~options.flat_foot
    for p = {'toe', 'heel'}
      pt = p{1};
      for b = {'lb', 'ub'}
        bound = b{1};
        footpos.(foot).(pt).(bound) = [footpos.(foot).(pt).(bound) footpos.(foot).(pt).(bound)(:,end)]; % check this...
        foottraj.(foot).(pt).(bound) = PPTrajectory(foh(ts, footpos.(foot).(pt).(bound)));
      end
    end
  end
  footsupport.(foot) = [footsupport.(foot) 1];

end

% creat ZMP trajectory
p = feetCenter(footpos.right.orig(:,end),footpos.left.orig(:,end));
zmp = [zmp,p(1:2)];
zmptraj = PPTrajectory(foh(ts,zmp));

% create support body trajectory
supporttraj = repmat(0*ts,length(biped.getLinkNames),1);
supporttraj(strcmp(biped.r_foot_name,biped.getLinkNames),:) = footsupport.right;
supporttraj(strcmp(biped.l_foot_name,biped.getLinkNames),:) = footsupport.left;
supporttraj = setOutputFrame(PPTrajectory(zoh(ts,supporttraj)),AtlasBody(biped));

end

