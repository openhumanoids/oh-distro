function [x_data, t] = robotLadderPlan(r_minimal,r, q0, qstar, comtraj, link_constraints,support_times,support)

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'robotLadderPlan');
nq = r.getNumDOF();

use_quasistatic_constraint =  true;
use_com_constraint =          true;
use_incr_com_constraint =     true;
use_final_com_constraint =    true;
use_arm_constraints =         true;
use_total_arm_constraints =   true;
use_pelvis_gaze_constraint =  true;
use_pelvis_constraint =       true;
use_utorso_constraint =       true;
use_knee_constraint =         true;
use_ankle_constraint =        true;
use_neck_constraint =         true;
use_collision_constraint =    false;
use_smoothing_constraint =    false;

pelvis = r.findLinkInd('pelvis');
utorso = r.findLinkInd('utorso');
r_foot = r.findLinkInd('r_foot');
l_foot = r.findLinkInd('l_foot');
neck_joint = findJointIndices(r,'neck');
%ankle_joints = [findJointIndices(r,'r_leg_aky')];
 ankle_joints = [findJointIndices(r,'l_leg_aky'); ...
                 findJointIndices(r,'r_leg_aky')];
% knee_joints = [findJointIndices(r,'l_leg_kny'); ...
%                 findJointIndices(r,'r_leg_kny')];
knee_joints = findJointIndices(r,'l_leg_kny');
arm_joints = findJointIndices(r,'arm');

n = 5;
shrink_factor = 0.9;
utorso_threshold = 25*pi/180;
pelvis_gaze_threshold = 20*pi/180;
ankle_limit = 20*pi/180*ones(size(ankle_joints));
knee_lb = 30*pi/180*ones(size(knee_joints));
knee_ub = inf*pi/180*ones(size(knee_joints));
hand_threshold = sin(15*pi/180);
hand_cone_threshold = sin(30*pi/180);
hand_pos_tol = 0.0;
%pelvis_threshold = 0.08;
pelvis_threshold = 0.05;
com_tol = 0.01;
com_incr_tol = 0.02;
com_tol_max = 0.2;
arm_tol = 6*pi/180/n;
arm_tol_total = 30*pi/180;
%comtraj = comtraj + 0.05;
com_tol_vec = [com_tol;com_tol;NaN];
com_incr_tol_vec = [com_incr_tol;com_incr_tol;NaN];
smoothing_tol = 1*pi/180;


% time spacing of samples for IK
tf = comtraj.tspan(2);
dt = 0.001;
ts = 0:dt:tf;
if length(ts)>1000 % limit number of IK samples to something reasonable
  ts = linspace(0,tf,1000);
  dt = ts(2)-ts(1);
end
nt = length(ts);
%ts(nt+1) = ts(end)+eps;
nt_support = length(support_times);
support_times(end+1) = support_times(end)+eps;

%% create desired joint trajectory
cost = Point(r.getStateFrame,1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 1000;
cost.base_pitch = 1000;
cost.base_yaw = 0;
cost.back_bkz = 1e2;
cost.back_bky = 1e2;
cost.back_bkx = 1e2;
cost = double(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setQ(diag(cost(1:r.getNumDOF)));
%ikoptions = ikoptions.setSequentialSeedFlag(true);
 %ikoptions = ikoptions.setMex(false);

msg ='Ladder Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
logical2str = @(b) regexprep(sprintf('%i',b),{'1','0'},{' ON ',' OFF '});
fprintf('\n');
fprintf('Constraint Summary\n');
fprintf('==================================\n');
fprintf('Name                      Status    Tolerance\n');
fprintf('QS Constraint             %s        %4.2f\n', logical2str(use_quasistatic_constraint),shrink_factor);
fprintf('Incr. COM Constraint:     %s        %4.2f m\n', logical2str(use_incr_com_constraint), com_incr_tol);
fprintf('COM Constraint:           %s        %4.2f m\n', logical2str(use_com_constraint), com_tol);
fprintf('Final COM Constraint:     %s\n', logical2str(use_final_com_constraint));
fprintf('Pelvis Constraint:        %s        %4.2f m\n', logical2str(use_pelvis_constraint), pelvis_threshold);
fprintf('Pelvis GazeConstraint:    %s        %4.2f deg\n', logical2str(use_pelvis_gaze_constraint),pelvis_gaze_threshold*180/pi);
fprintf('Torso Constraint:         %s        %4.2f deg\n', logical2str(use_utorso_constraint),utorso_threshold*180/pi);
fprintf('Knee Constraint:          %s        [%4.2f deg, %4.2f deg]\n', logical2str(use_knee_constraint),knee_lb,knee_ub);
fprintf('Neck Constraint:          %s\n', logical2str(use_neck_constraint));
fprintf('Ankle Constraint:         %s\n', logical2str(use_ankle_constraint));
fprintf('Arm Constraints (incr):   %s        %4.2f deg\n', logical2str(use_arm_constraints),arm_tol*180/pi);
fprintf('Arm Constraints (total):  %s        %4.2f deg\n', logical2str(use_total_arm_constraints),arm_tol_total*180/pi);

% v = r.constructVisualizer;
% v.display_dt = 0.05;
q = zeros(nq,nt);
q(:,1) = q0;

r_foot_support_data = zeros(size(support_times));
r_foot_support_data(1:end-1) = cellfun(@(supp) double(any(supp.bodies==r_foot)),support); 
r_foot_support_data(end) = r_foot_support_data(end-1);
r_foot_support_traj = PPTrajectory(zoh(support_times,r_foot_support_data));

l_foot_support_data = zeros(size(support_times));
l_foot_support_data(1:end-1) = cellfun(@(supp) double(any(supp.bodies==l_foot)),support); 
l_foot_support_data(end) = l_foot_support_data(end-1);
l_foot_support_traj = PPTrajectory(zoh(support_times,l_foot_support_data));
basic_constraints = {};

if use_ankle_constraint
  ankle_constraint = PostureConstraint(r,tf*[0*0.9,Inf]);
  ankle_constraint = ankle_constraint.setJointLimits(ankle_joints, ...
    -ankle_limit, ...
    10*ankle_limit); 
  basic_constraints = [basic_constraints, {ankle_constraint}];
end

if use_knee_constraint
  knee_constraint = PostureConstraint(r);
  knee_constraint = knee_constraint.setJointLimits(knee_joints, ...
    knee_lb, ...
    knee_ub);
 basic_constraints = [basic_constraints, {knee_constraint}];
end

if use_neck_constraint
  neck_constraint = PostureConstraint(r);
  neck_constraint = neck_constraint.setJointLimits(neck_joint,q0(neck_joint),q0(neck_joint));
  basic_constraints = [basic_constraints, {neck_constraint}];
end

if use_total_arm_constraints %|| length(link_constraints)==2
  arm_constraint = PostureConstraint(r);
  arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-arm_tol_total,q0(arm_joints)+arm_tol_total);
  basic_constraints = [basic_constraints, {arm_constraint}];
end

if use_collision_constraint
  basic_constraints = [ ...
    basic_constraints
    {AllBodiesClosestDistanceConstraint(r,0.05,1e3)}];
end

if use_utorso_constraint
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldGazeOrientConstraint(r,utorso,[0;0;1],[1;0;0;0],utorso_threshold,90*pi/180)}];
end

if use_pelvis_gaze_constraint
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldGazeOrientConstraint(r,pelvis,[0;0;1],[1;0;0;0],pelvis_gaze_threshold,90*pi/180)}];
end

kinsol = doKinematics(r,q0);
pelvis_xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
o_T_pelvis = HT(pelvis_xyzrpy(1:3),pelvis_xyzrpy(4),pelvis_xyzrpy(5),pelvis_xyzrpy(6));
o_T_pelvis(1:3,1:3) = eye(3);
if use_pelvis_constraint
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldPositionInFrameConstraint(r,pelvis, ...
    [0;0;0], o_T_pelvis, [NaN;-pelvis_threshold;NaN], ...
    [NaN;pelvis_threshold;NaN])}];
end

q_seed = q0;
q_nom = qstar;
rpy_tol_max = 30*pi/180;
rpy_tol_traj = PPTrajectory(foh(linspace(0,tf,5),repmat([0,0,rpy_tol_max,0,0],3,1)));
for i=1:length(link_constraints)
  if ~isempty(link_constraints(i).traj)
    deriv = fnder(link_constraints(i).traj);
    t_moving = ts(any(eval(deriv,ts) > 0,1));
    if isempty(t_moving)
      link_constraints(i).rpy_tol_traj = PPTrajectory(foh([0,tf],repmat([0,0],3,1)));
    else
      t_move0 = t_moving(1);
      t_movef = t_moving(end);
      link_constraints(i).rpy_tol_traj = PPTrajectory(foh([0,linspace(t_move0,t_movef,3),tf],repmat([0,0,rpy_tol_max,0,0],3,1)));
    end
  end
end
com_fade_traj = PPTrajectory(foh(linspace(0,tf,4),0.2*[0,1,1,0]));

msg = '  0%%';
fprintf(['Progress: ',msg]);
len_prev_msg = length(sprintf(msg));
n_err = 0;
first_err = true;
err_segments = [];
[joint_min,joint_max] = getJointLimits(r);
for i=1:nt
  t = ts(i);
  constraints = basic_constraints;
  
  r_foot_supported = eval(r_foot_support_traj,t);
  l_foot_supported = eval(l_foot_support_traj,t);
  if use_quasistatic_constraint && (r_foot_supported || l_foot_supported)
    qsc = QuasiStaticConstraint(r);
    if r_foot_supported
      qsc = qsc.addContact(r_foot,r.getBodyContacts(r_foot));
    end
    if l_foot_supported
      qsc = qsc.addContact(l_foot,r.getBodyContacts(l_foot));
    end
    qsc = qsc.setShrinkFactor(shrink_factor);
    qsc = qsc.setActive(true);
    constraints = [constraints, {qsc}];
  end
  if use_arm_constraints
    arm_constraint = PostureConstraint(r);
    if i==1
      arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-arm_tol,q0(arm_joints)+arm_tol);
    else
      arm_constraint = arm_constraint.setJointLimits(arm_joints,q(arm_joints,i-1)-arm_tol,q(arm_joints,i-1)+arm_tol);
    end
    constraints = [constraints, {arm_constraint}];
  end

  if use_smoothing_constraint
    smoothing_constraint = PostureConstraint(r);
    if i==1
      smoothing_constraint = smoothing_constraint.setJointLimits((1:nq)', ...
        max(q0-smoothing_tol,joint_min), ...
        min(q0+smoothing_tol,joint_max-1e-6));
    else
      smoothing_constraint = smoothing_constraint.setJointLimits(1:nq, ...
        max(q(:,i-1)-smoothing_tol,joint_min), ...
        min(q(:,i-1)+smoothing_tol,joint_max-1e-6));
    end
    constraints = [basic_constraints, {smoothing_constraint}];
  end
  com = eval(comtraj,ts(i));
  com(3) = NaN;
%   com_array = repmat(com,1,4);
  for j = 1:length(link_constraints)
    if ~isempty(link_constraints(j).traj)
      pos_eq = link_constraints(j).traj.eval(t);
      rpy_tol = eval(link_constraints(j).rpy_tol_traj,t);
      constraints = [ ...
        constraints, ...
        {WorldPositionConstraint(r,link_constraints(j).link_ndx, ...
            link_constraints(j).pt(1:3), ...
            pos_eq(1:3),pos_eq(1:3)),...
         WorldEulerConstraint(r,link_constraints(j).link_ndx, ...
            pos_eq(4:6)-rpy_tol, ...
            pos_eq(4:6)+rpy_tol)}];
    else
      %pos_min = link_constraints(j).min_traj.eval(t)-0.1;
      %pos_max = link_constraints(j).max_traj.eval(t)+0.1;
      pos_min = link_constraints(j).min_traj.eval(t);
      pos_max = link_constraints(j).max_traj.eval(t);
      pos_eq = (pos_min+pos_max)/2;
      rpy = quat2rpy(pos_eq(4:7));
      pelvis_T_rail = eye(4);
      pelvis_T_rail(1:3,1:3) = roty(30*pi/180);
      pelvis_T_rail(1:3,4) = pos_eq(1:3) - o_T_pelvis(1:3,4);
      o_T_rail = o_T_pelvis*pelvis_T_rail;
      constraints = [ ...
        constraints, ...
        {WorldPositionInFrameConstraint(r, link_constraints(j).link_ndx, ...
          link_constraints(j).pt, o_T_rail, [-hand_pos_tol;-hand_pos_tol;-hand_pos_tol], [hand_pos_tol;hand_pos_tol;hand_pos_tol]), ...
         WorldGazeOrientConstraint(r,link_constraints(j).link_ndx,link_constraints(j).axis,pos_min(4:7),hand_cone_threshold,hand_threshold)}];
         %WorldQuatConstraint(r,link_constraints(j).link_ndx,pos_min(4:7),hand_threshold)}];
%       com_array(1:2,j) = (pos_min(1:2)+pos_max(1:2))/2;
    end
  end
%   com = eval(com_fade_traj,t)*mean(com_array,2) + (1-eval(com_fade_traj,t))*com;

  if i > 1 && use_incr_com_constraint
    constraints = [ ...
      constraints, ...
      {WorldCoMConstraint(r,com_prev-com_incr_tol_vec,com_prev+com_incr_tol_vec)}];
  end  
  if use_com_constraint
    constraints = [ ...
      constraints, ...
      {WorldCoMConstraint(r,com-com_tol_vec,com+com_tol_vec)}];
  end
  [q(:,i),info] = inverseKinPointwise(r,t,q_seed,q_nom,constraints{:},ikoptions);
%   if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
  if info ~= 1 
    if use_com_constraint
      com_tol_local = com_tol;
      while info ~= 1 && com_tol_local < com_tol_max
        %display(com_tol_local(1))
        com_tol_local = com_tol_local+0.01;
        constraints{end} = WorldCoMConstraint(r,com-com_tol_local,com+com_tol_local);
        [q(:,i),info] = inverseKinPointwise(r,t,q_seed,q_nom,constraints{:},ikoptions);
      end
    end
    if info ~= 1 
      n_err = n_err+1; 
      if first_err
        err_segments(end+1,1) = i/nt;
        first_err = false;
      else
        err_segments(end,2) = i/nt;
      end
    else
      first_err = true;
    end
  else
    first_err = true;
  end;
  msg = '%3.0f%% (No. Errors: %4.0f)';
  fprintf([repmat('\b',1,len_prev_msg), msg],i/nt*100,n_err);
  len_prev_msg = length(sprintf(msg,i/nt*100,n_err));
  if i==1
    % Compute plan from q0 to q(:,1)
    nt_init = floor(nt/4);
    t_init = 0:dt:nt_init*dt;
    q_init_nom = PPTrajectory(foh([t_init(1),t_init(end)],[q0,q(:,1)]));
    [q_init,info] = inverseKinPointwise(r,t_init, ...
                                        eval(q_init_nom,t_init), ...
                                        eval(q_init_nom,t_init), ...
                                        constraints{:},ikoptions);
  end
  q_seed = q(:,i);
  kinsol = doKinematics(r,q_seed);
  com_prev = r.getCOM(kinsol);
  com_prev(3) = NaN;
%   q_nom = q(:,i);
  %   disp(info);
end
fprintf('\n');
% q_data = q;
% t = ts;
q_data = [q_init,q];
t = [t_init, ts+t_init(end)+dt];
if use_final_com_constraint
  % Compute plan from q(:,end) to qf
  nt_end = floor(nt/4);
  t_end = 0:dt:nt_init*dt;
  t_end_coarse = linspace(t_end(1),t_end(end),3);
%   com_constraint_f = WorldCoMConstraint(r,com,com,t_end(end)*[1,1]);
  com_constraint_f = QuasiStaticConstraint(r);
  com_constraint_f = com_constraint_f.setActive(true);
  com_constraint_f = com_constraint_f.setShrinkFactor(0.5);
  foot1_pts = r.getBodyContacts(link_constraints(1).link_ndx);
  foot2_pts = r.getBodyContacts(link_constraints(2).link_ndx);
  com_constraint_f = com_constraint_f.addContact(link_constraints(1).link_ndx,foot1_pts,link_constraints(2).link_ndx,foot2_pts);
  
  [qf,info] = inverseKin(r,qstar,qstar,constraints{1:end},com_constraint_f,ikoptions);
  if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
  q_end_nom = PPTrajectory(foh([t_end(1),t_end(end)],[q(:,end),qf]));
  end_posture_constraint = PostureConstraint(r,t_end_coarse(end)*[1,1]);
  end_posture_constraint = end_posture_constraint.setJointLimits((1:nq)',qf,qf);
  % [q_end,info] = inverseKinPointwise(r,t_end, ...
  %   eval(q_end_nom,t_end), ...
  %   eval(q_end_nom,t_end), ...
  %   constraints{:},ikoptions);
  [x_end_traj,info] = inverseKinTraj(r,t_end_coarse, ...
    q_end_nom, ...
    q_end_nom, ...
    constraints{:},end_posture_constraint,ikoptions);
  x_end = eval(x_end_traj,t_end);
  q_end = x_end(1:nq,:);
  % q_end=[];
  % [q(:,2:end),info] = inverseKinPointwise(r,ts(2:end),repmat(q0,1,nt-1),repmat(qstar,1,nt-1),constraints{:},ikoptions);
  % q = q(:,1:5:end);
  q_data = [q_data,q_end];
  t = [t, t_end + t(end) + dt];
end
for i = 1:size(q_data,2)
  kinsol = doKinematics(r,q_data(:,i));
  com = getCOM(r,kinsol);
  com(3) = 0;
  phi = i/size(q_data,2);
  lcmgl.glColor3f(phi,0,1-phi);
  lcmgl.sphere(com,0.02,20,20);
end
lcmgl.switchBuffers();
%x_data = zeros(2*nq,size(q_data,2));
%x_data(1:getNumDOF(r),:) = q_data;
display(err_segments);
t = t(1:n:end);
x_data = zeros(2*nq,length(t));
for i = 1:nq
  x_data(i,:) = decimate(q_data(i,:)',n);
end
%ts = ts(1:end-1)
end
