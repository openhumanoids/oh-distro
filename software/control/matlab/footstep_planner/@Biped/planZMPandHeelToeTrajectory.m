function [zmptraj,foottraj,step_times,supporttraj] = planZMPandHeelToeTrajectory(biped,q0, Xright, Xleft,step_time,options)
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

step_locations = struct('right', Xright(1:6,:), 'left', Xleft(1:6,:));

kinsol = doKinematics(biped,q0);

foot_body = struct('right', findLink(biped, biped.r_foot_name),...
  'left', findLink(biped, biped.l_foot_name));

com0 = getCOM(biped,q0);
foot0 = struct('right', forwardKin(biped,kinsol,foot_body.right,[0;0;0],true),...
  'left', forwardKin(biped,kinsol,foot_body.left,[0;0;0],true));

function fpos = footPoint(foot, grp, pos)
  fpos = biped.footOrig2Contact(pos, grp, strcmp(foot, 'right'));
  fpos = fpos(1:3, :);
end

function pos = feetCenter(rfootpos,lfootpos)
  rcen = biped.footOrig2Contact(rfootpos, 'center', 1);
  lcen = biped.footOrig2Contact(lfootpos, 'center', 0);
  % rcen = footPoint('right', 'center', rfootpos);
  % lcen = footPoint('left', 'center', lfootpos);
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
        footpos.(foot).(grp).(bound) = biped.footOrig2Contact(footpos.(foot).orig, grp, strcmp(foot, 'right'));
        % footpos.(foot).(grp).(bound) = footPoint(foot, grp, footpos.(foot).orig);
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
  
  step.(m_foot).orig = interp1([0; 1], [step_locations.(m_foot)(1:6, istep.(m_foot)), step_locations.(m_foot)(1:6, istep.(m_foot)+1)]', [0, .5, 1, 1, 1]')';
  step.(m_foot).orig(3,:) = step.(m_foot).orig(3,:) + [0, 0.05, 0, 0, 0];
  
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
    % Shift the ZMP by 2cm closer to the center of the feet
    foot_center = footPoint(s_foot, 'center', step.(s_foot).orig(:,1));
    step_center = biped.footCenter2StepCenter(foot_center, strcmp(s_foot, 'right'));
    zmp_shift = (step_center(1:2) - foot_center(1:2));
    zmp_shift = zmp_shift ./ sqrt(sum(zmp_shift.^2)) * 0.02; 
    stepzmp = [repmat(footPoint(s_foot, 'center', step.(s_foot).orig(:,1)),1,3)+zmp_shift,...
               repmat(feetCenter(step.(m_foot).orig(:,end), step.(s_foot).orig(:,end)), 1, 2)];
    disp('s_foot center')
    footPoint(s_foot, 'center', step.(s_foot).orig(:))
    disp('m_foot center')
    footPoint(m_foot, 'center', step.(m_foot).orig(:))
    disp('step zmp')
    stepzmp
  end
    
  istep.(m_foot) = istep.(m_foot) + 1;
  
%   footsupport.(m_foot) = [footsupport.(m_foot) 0 0 .5 1 1];
  footsupport.(m_foot) = [footsupport.(m_foot) 0 0 1 1 1]; 
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

