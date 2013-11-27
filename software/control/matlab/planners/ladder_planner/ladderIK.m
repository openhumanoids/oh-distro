function [q_data, t_data, ee_info] = ladderIK(r,ts,q0,qstar,ee_info,ladder_opts,ikoptions)
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'robotLadderPlan');
red = {1,0,0};
blue = {0,0,1};
gray = {0.5,0.5,0.5};
black = {0,0,0};
nq = r.getNumDOF();
F_hand_max = 25; %lbs

pelvis = r.findLinkInd('pelvis');
utorso = r.findLinkInd('utorso');
neck_joint = findJointIndices(r,'neck');
%ankle_joints = [findJointIndices(r,'r_leg_aky')];
 ankle_joints = [findJointIndices(r,'l_leg_aky'); ...
                 findJointIndices(r,'r_leg_aky')];
% knee_joints = [findJointIndices(r,'l_leg_kny'); ...
%                 findJointIndices(r,'r_leg_kny')];
knee_joints = findJointIndices(r,'l_leg_kny');
arm_joints = findJointIndices(r,'arm');

if ~isfield(ladder_opts,'use_quasistatic_constraint') 
  ladder_opts.use_quasistatic_constraint =  true;
end
if ~isfield(ladder_opts,'use_com_constraint') 
  ladder_opts.use_com_constraint =          false;
end
if ~isfield(ladder_opts,'use_incr_com_constraint') 
  ladder_opts.use_incr_com_constraint =     false;
end
if ~isfield(ladder_opts,'use_final_com_constraint') 
  ladder_opts.use_final_com_constraint =    true;
end
if ~isfield(ladder_opts,'use_arm_constraints') 
  ladder_opts.use_arm_constraints =         false;
end
if ~isfield(ladder_opts,'use_total_arm_constraints') 
  ladder_opts.use_total_arm_constraints =   false;
end
if ~isfield(ladder_opts,'use_pelvis_gaze_constraint') 
  ladder_opts.use_pelvis_gaze_constraint =  true;
end
if ~isfield(ladder_opts,'use_pelvis_constraint') 
  ladder_opts.use_pelvis_constraint =       true;
end
if ~isfield(ladder_opts,'use_utorso_constraint') 
  ladder_opts.use_utorso_constraint =       true;
end
if ~isfield(ladder_opts,'use_knee_constraint') 
  ladder_opts.use_knee_constraint =         true;
end
if ~isfield(ladder_opts,'use_ankle_constraint') 
  ladder_opts.use_ankle_constraint =        true;
end
if ~isfield(ladder_opts,'use_neck_constraint') 
  ladder_opts.use_neck_constraint =         true;
end
if ~isfield(ladder_opts,'use_swing_foot_euler_constraint') 
  ladder_opts.use_swing_foot_euler_constraint =         false;
end
if ~isfield(ladder_opts,'compute_intro')
  ladder_opts.compute_intro = false;
end
if ~isfield(ladder_opts,'smooth_output');
  ladder_opts.smooth_output = false;
end
if ~isfield(ladder_opts,'smoothing_span');
  ladder_opts.smoothing_span = 5;
end
if ~isfield(ladder_opts,'smoothing_method');
  ladder_opts.smoothing_method = 'moving';
end
if ~isfield(ladder_opts,'use_collision_constraint') 
  ladder_opts.use_collision_constraint =    false;
end
if ~isfield(ladder_opts,'use_smoothing_constraint') 
  ladder_opts.use_smoothing_constraint =    false;
end

if ~isfield(ladder_opts,'n') 
  ladder_opts.n = 1;
end
if ~isfield(ladder_opts,'shrink_factor') 
  ladder_opts.shrink_factor = 0.1;
end
if ~isfield(ladder_opts,'final_shrink_factor') 
  ladder_opts.shrink_factor = 0.5;
end
if ~isfield(ladder_opts,'utorso_threshold') 
  ladder_opts.utorso_threshold = 25*pi/180;
end
if ~isfield(ladder_opts,'pelvis_gaze_threshold') 
  ladder_opts.pelvis_gaze_threshold = 20*pi/180;
end
if ~isfield(ladder_opts,'ankle_limit') 
  ladder_opts.ankle_limit = 25*pi/180*ones(size(ankle_joints));
end
if ~isfield(ladder_opts,'knee_lb') 
  ladder_opts.knee_lb = 30*pi/180*ones(size(knee_joints));
end
if ~isfield(ladder_opts,'knee_ub') 
  ladder_opts.knee_ub = inf*pi/180*ones(size(knee_joints));
end
if ~isfield(ladder_opts,'hand_threshold') 
  ladder_opts.hand_threshold = sin(1*pi/180);
end
if ~isfield(ladder_opts,'hand_cone_threshold') 
  ladder_opts.hand_cone_threshold = sin(1*pi/180);
end
if ~isfield(ladder_opts,'hand_pos_tol') 
  ladder_opts.hand_pos_tol = 0.01;
end
if ~isfield(ladder_opts,'pelvis_threshold') 
  ladder_opts.pelvis_threshold = 0.05;
end
if ~isfield(ladder_opts,'com_tol') 
  ladder_opts.com_tol = 0.1;
end
if ~isfield(ladder_opts,'com_incr_tol') 
  ladder_opts.com_incr_tol = 0.02;
end
if ~isfield(ladder_opts,'com_tol_max') 
  ladder_opts.com_tol_max = 0.5;
end
if ~isfield(ladder_opts,'qs_margin') 
  ladder_opts.qs_margin = 0.0;
end
if ~isfield(ladder_opts,'arm_tol') 
  ladder_opts.arm_tol = 6*pi/180/n;
end
if ~isfield(ladder_opts,'arm_tol_total') 
  ladder_opts.arm_tol_total = 30*pi/180;
end
com_tol_vec = [ladder_opts.com_tol;ladder_opts.com_tol;NaN];
com_incr_tol_vec = [ladder_opts.com_incr_tol;ladder_opts.com_incr_tol;NaN];

% time spacing of samples for IK
if ladder_opts.n > 1
  ts = linspace(ts(1),ts(end),length(ts)*ladder_opts.n);
end
nt = length(ts);

msg ='Ladder Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
logical2str = @(b) regexprep(sprintf('%i',b),{'1','0'},{' ON ',' OFF '});
fprintf('\n');
fprintf('Constraint Summary\n');
fprintf('==================================\n');
fprintf('Name                      Status    Tolerance\n');
fprintf('QS Constraint             %s        %4.2f\n', logical2str(ladder_opts.use_quasistatic_constraint),ladder_opts.qs_margin);
fprintf('Incr. COM Constraint:     %s        %4.2f m\n', logical2str(ladder_opts.use_incr_com_constraint), ladder_opts.com_incr_tol);
fprintf('COM Constraint:           %s        %4.2f m\n', logical2str(ladder_opts.use_com_constraint), ladder_opts.com_tol);
fprintf('Final COM Constraint:     %s\n', logical2str(ladder_opts.use_final_com_constraint));
fprintf('Pelvis Constraint:        %s        %4.2f m\n', logical2str(ladder_opts.use_pelvis_constraint), ladder_opts.pelvis_threshold);
fprintf('Pelvis GazeConstraint:    %s        %4.2f deg\n', logical2str(ladder_opts.use_pelvis_gaze_constraint),ladder_opts.pelvis_gaze_threshold*180/pi);
fprintf('Torso Constraint:         %s        %4.2f deg\n', logical2str(ladder_opts.use_utorso_constraint),ladder_opts.utorso_threshold*180/pi);
fprintf('Knee Constraint:          %s        [%4.2f deg, %4.2f deg]\n', logical2str(ladder_opts.use_knee_constraint),ladder_opts.knee_lb,ladder_opts.knee_ub);
fprintf('Neck Constraint:          %s\n', logical2str(ladder_opts.use_neck_constraint));
fprintf('Ankle Constraint:         %s\n', logical2str(ladder_opts.use_ankle_constraint));
fprintf('Arm Constraints (incr):   %s        %4.2f deg\n', logical2str(ladder_opts.use_arm_constraints),ladder_opts.arm_tol*180/pi);
fprintf('Arm Constraints (total):  %s        %4.2f deg\n', logical2str(ladder_opts.use_total_arm_constraints),ladder_opts.arm_tol_total*180/pi);

% v = r.constructVisualizer;
% v.display_dt = 0.05;

hand_grasped(1) = ~isempty(ee_info.hands(1));
hand_grasped(2) = ~isempty(ee_info.hands(1));

basic_constraints = {};

tf = ts(end);

if ladder_opts.use_ankle_constraint
  ankle_constraint = PostureConstraint(r,tf*[0*0.9,Inf]);
  ankle_constraint = ankle_constraint.setJointLimits(ankle_joints, ...
    -ladder_opts.ankle_limit, ...
    10*ladder_opts.ankle_limit); 
  basic_constraints = [basic_constraints, {ankle_constraint}];
end

if ladder_opts.use_knee_constraint
  knee_constraint = PostureConstraint(r);
  knee_constraint = knee_constraint.setJointLimits(knee_joints, ...
    ladder_opts.knee_lb, ...
    ladder_opts.knee_ub);
 basic_constraints = [basic_constraints, {knee_constraint}];
end

if ladder_opts.use_neck_constraint
  neck_constraint = PostureConstraint(r);
  neck_constraint = neck_constraint.setJointLimits(neck_joint,q0(neck_joint),q0(neck_joint));
  basic_constraints = [basic_constraints, {neck_constraint}];
end

if ladder_opts.use_total_arm_constraints %|| length(link_constraints)==2
  arm_constraint = PostureConstraint(r);
  arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-ladder_opts.arm_tol_total,q0(arm_joints)+ladder_opts.arm_tol_total);
  basic_constraints = [basic_constraints, {arm_constraint}];
end

if ladder_opts.use_collision_constraint
  basic_constraints = [ ...
    basic_constraints
    {AllBodiesClosestDistanceConstraint(r,0.05,1e3)}];
end

kinsol = doKinematics(r,q0);
if ladder_opts.use_utorso_constraint
  utorso_posquat = forwardKin(r,kinsol,utorso,[0;0;0],2);
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldGazeOrientConstraint(r,utorso,[0;0;1],utorso_posquat(4:7),ladder_opts.utorso_threshold,90*pi/180)}];
end

if ladder_opts.use_pelvis_gaze_constraint
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldGazeOrientConstraint(r,pelvis,[0;0;1],[1;0;0;0],ladder_opts.pelvis_gaze_threshold,90*pi/180)}];
end

pelvis_xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
o_T_pelvis = HT(pelvis_xyzrpy(1:3),pelvis_xyzrpy(4),pelvis_xyzrpy(5),pelvis_xyzrpy(6));
o_T_pelvis(1:3,1:3) = eye(3);
if ladder_opts.use_pelvis_constraint
  basic_constraints = [ ...
    basic_constraints, ...
    {WorldPositionInFrameConstraint(r,pelvis, ...
    [0;0;0], o_T_pelvis, [NaN;-ladder_opts.pelvis_threshold;NaN], ...
    [NaN;ladder_opts.pelvis_threshold;NaN])}];
end

rpy_tol_max = 30*pi/180;
for i=1:2
  deriv = fnder(ee_info.feet(i).traj);
  t_moving = ts(any(eval(deriv,ts) > 0,1));
  if isempty(t_moving)
    ee_info.feet(i).rpy_tol_traj = PPTrajectory(foh([0,tf],repmat([0,0],3,1)));
  else
    t_move0 = t_moving(1);
    t_movef = t_moving(end);
    ee_info.feet(i).rpy_tol_traj = ...
      PPTrajectory(foh([0,linspace(t_move0,t_movef,3),tf],repmat([0,0,rpy_tol_max,0,0],3,1)));
  end
end

msg = '  0%%';
fprintf(['Progress: ',msg]);
len_prev_msg = length(sprintf(msg));
n_err = 0;
first_err = true;
err_segments = [];

q_seed = q0;
q_nom = qstar;
q = zeros(nq,nt);
q(:,1) = q_seed;
constraint_array = cell(nt,1);
for i=1:nt
  t_data = ts(i);
  constraints = basic_constraints;
  
  foot_supported(1) = ( eval(ee_info.feet(1).support_traj,t_data) && eval(ee_info.feet(1).support_traj,ts(min(i+1,nt))) );
  foot_supported(2) = ( eval(ee_info.feet(2).support_traj,t_data) && eval(ee_info.feet(2).support_traj,ts(min(i+1,nt))) );
  if ladder_opts.use_quasistatic_constraint && any(foot_supported)
    %qsc = QuasiStaticConstraint(r);
    foot_pts_in_world = [];
    hand_pts_in_world = [];
    for j = 1:2
      if foot_supported(j)
        foot_pts = r.getBodyContacts(ee_info.feet(j).idx);
        foot_pos = ee_info.feet(j).traj.eval(t_data);
        T_foot_to_world = [rpy2rotmat(foot_pos(4:6)),foot_pos(1:3); ... 
          zeros(1,3),1];
        curr_foot_pts_in_world = T_foot_to_world*[foot_pts;ones(1,size(foot_pts,2))];
        foot_pts_in_world = [foot_pts_in_world, curr_foot_pts_in_world(1:3,:)];
        %qsc = qsc.addContact(foot,r.getBodyContacts(foot));
      end
      if hand_grasped(j)
        hand_pos = 0.5*(ee_info.hands(j).min_traj.eval(t_data) ...
          +ee_info.hands(j).min_traj.eval(t_data));
        hand_pts_in_world = [hand_pts_in_world, hand_pos(1:3,:)];
      end
    end
    hand_pts_in_world = circshift(hand_pts_in_world, [0 1]);
    K = convhulln(foot_pts_in_world(1:2,:)');
    foot_chull_pts_in_world = foot_pts_in_world(:,K(:,1));
    foot_chull_pts_in_world(3,:) = min(foot_chull_pts_in_world(3,:));
    
    n_foot_chull_pts = size(foot_chull_pts_in_world,2);
    foot_chull_pts_in_world = foot_chull_pts_in_world*ladder_opts.shrink_factor+bsxfun(@times,mean(foot_chull_pts_in_world,2)*(1-ladder_opts.shrink_factor),ones(1,n_foot_chull_pts));
    
    foot_chull_pts_in_bot = ...
      o_T_pelvis\[foot_chull_pts_in_world;ones(1,size(foot_chull_pts_in_world,2))];
    foot_chull_pts_in_bot(4,:) = [];
    foot_pts_in_bot = ...
      o_T_pelvis\[foot_pts_in_world;ones(1,size(foot_pts_in_world,2))];
    foot_pts_in_bot(4,:) = [];
    n_chull_pts = size(foot_chull_pts_in_world,2);
    foot_chull_edges = circshift(foot_chull_pts_in_world,[0,1]) ...
                       - foot_chull_pts_in_world;
    foot_chull_edge_directions = ...
      bsxfun(@rdivide,foot_chull_edges, ...
             sqrt(sum(foot_chull_edges.^2,1)));
    if ~isempty(hand_pts_in_world)
      hand_pts_in_bot = ...
        o_T_pelvis\[hand_pts_in_world;ones(1,size(hand_pts_in_world,2))];
      hand_pts_in_bot(4,:) = [];
      foot_back = [min(foot_pts_in_bot(1,:));mean(foot_pts_in_bot(2:3,:),2)];
      hand_mean = mean(hand_pts_in_bot,2);
      l_h = dot(hand_mean-foot_back,[cos(pi/3);0;sin(pi/3)]);
      m = r.getMass();
      l_c = l_h/(9.81*m)*F_hand_max*4.4;
      if size(hand_pts_in_world,2)
        h1_minus_chull_pts = ...
          bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,1));
        h1_angles = atan2(h1_minus_chull_pts(2,:),h1_minus_chull_pts(1,:));
        [~,p1_idx] = min(h1_angles);
        foot_chull_pts_in_world = ...
          circshift(foot_chull_pts_in_world,[0 -p1_idx+1]);
        foot_chull_pts_in_bot = ...
          circshift(foot_chull_pts_in_bot,[0 -p1_idx+1]);
        foot_chull_edges = circshift(foot_chull_edges,[0 -p1_idx+1]);
        foot_chull_edge_directions = ...
          circshift(foot_chull_edge_directions,[0 -p1_idx+1]);
        p1_idx = 1;
        h2_minus_chull_pts = ...
          bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,2));
        h2_angles = atan2(h2_minus_chull_pts(2,:),h2_minus_chull_pts(1,:));
        [~,p2_idx] = max(h2_angles);

        % Replace edge(p1_idx) with vector from h1 to p1
%         foot_chull_edge_directions(:,p1_idx) = ...
%           -normalizeVec(foot_chull_pts_in_world(:,p1_idx) - [hand_pts_in_world(1:2,1); foot_chull_pts_in_world(3,p1_idx)]);
        foot_chull_edge_directions = ...
          [foot_chull_edge_directions(:,1), ...
          -normalizeVec(foot_chull_pts_in_world(:,p1_idx) - [hand_pts_in_world(1:2,1); foot_chull_pts_in_world(3,p1_idx)]),...
          -normalizeVec([hand_pts_in_world(1:2,2);foot_chull_pts_in_world(3,p2_idx)] - foot_chull_pts_in_world(:,p2_idx)),...
          foot_chull_edge_directions(:,p2_idx+1:end)];
        foot_chull_pts_in_world = ...
          foot_chull_pts_in_world(:,[p1_idx,p1_idx,p2_idx:end]);
%           foot_chull_pts_in_world(:,[1,p2_idx,p2_idx:end]);
        p2_idx = 3;
      elseif size(hand_hold_pts,1)
        h1_minus_chull_pts = ...
          bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,1));
        h1_angles = atan2(h1_minus_chull_pts(2,:),h1_minus_chull_pts(1,:));
        [~,p1_idx] = min(h1_angles);
        foot_chull_pts_in_world = ...
          circshift(foot_chull_pts_in_world,[0 -p1_idx]);
        foot_chull_pts_in_bot = ...
          circshift(foot_chull_pts_in_bot,[0 -p1_idx]);
        p1_idx = 1;
      end
    end
    for j = 1:size(foot_chull_edge_directions)
      R = quat2rotmat(quatTransform([1;0;0],foot_chull_edge_directions(:,j)));
      P = foot_chull_pts_in_world(:,j);
      T = [R,P;zeros(1,3),1];
      com_halfspace_constraint = ...
        WorldCoMInFrameConstraint(r,T,[NaN;ladder_opts.qs_margin;NaN],[NaN;NaN;NaN]);
      constraints = [constraints, {com_halfspace_constraint}];
    end
    R = o_T_pelvis(1:3,1:3);
    P = mean(foot_back,2);
    T = [R,P;zeros(1,3),1];
    com_halfspace_constraint = ...
      WorldCoMInFrameConstraint(r,T,[-l_c;NaN;NaN],[NaN;NaN;NaN]);
    constraints = [constraints, {com_halfspace_constraint}];

    % Draw all support points
    lcmgl.glColor3f(gray{:});
    for pt = foot_pts_in_world
      lcmgl.sphere(pt,0.01,20,20);
    end
    % Draw convex hull points
    lcmgl.glColor3f(blue{:});
    for pt = foot_chull_pts_in_world
      lcmgl.sphere(pt,0.02,20,20);
    end
    % Draw edges
    %lcmgl.glColor3f(black{:});
    %lcmgl.glLineWidth(4);
    %for j = 1:n_chull_pts
      %lcmgl.glBegin(lcmgl.LCMGL_LINES);
      %lcmgl.glVertex3d(foot_chull_pts_in_world(1,j),foot_chull_pts_in_world(2,j),foot_chull_pts_in_world(3,j))
      %lcmgl.glVertex3d(foot_chull_pts_in_world(1,j)+foot_chull_edges(1,j),foot_chull_pts_in_world(2,j)+foot_chull_edges(2,j),foot_chull_pts_in_world(3,j)+foot_chull_edges(3,j))
      %lcmgl.glEnd()
    %end
    % Draw hand points
    lcmgl.glColor3f(red{:});
    for pt = hand_pts_in_world
      pt(3) = 0;
      lcmgl.sphere(pt,0.02,20,20);
    end
    lcmgl.glColor3f(black{:});
    lcmgl.sphere(foot_chull_pts_in_world(:,p1_idx),0.03,20,20);
    lcmgl.glColor3f(gray{:});
    lcmgl.sphere(foot_chull_pts_in_world(:,p2_idx),0.03,20,20);
    lcmgl.glLineWidth(2);
    for j = 1:size(foot_chull_edge_directions,2)
      aa = quat2axis(quatTransform([1;0;0],foot_chull_edge_directions(:,j)));
      lcmgl.glTranslated(foot_chull_pts_in_world(1,j), ...
                        foot_chull_pts_in_world(2,j), ...
                        foot_chull_pts_in_world(3,j));
      lcmgl.glRotated(-aa(4)*180/pi,aa(1),aa(2),aa(3));
      lcmgl.glDrawAxes();
      lcmgl.glRotated(aa(4)*180/pi,aa(1),aa(2),aa(3));
      lcmgl.glTranslated(-foot_chull_pts_in_world(1,j), ...
                        -foot_chull_pts_in_world(2,j), ...
                        -foot_chull_pts_in_world(3,j));
    end
    lcmgl.switchBuffers();

    %qsc = qsc.setShrinkFactor(ladder_opts.shrink_factor);
    %qsc = qsc.setActive(true);
    %constraints = [constraints, {qsc}];
  end
  if ladder_opts.use_arm_constraints
    arm_constraint = PostureConstraint(r);
    if i==1
      arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-ladder_opts.arm_tol,q0(arm_joints)+ladder_opts.arm_tol);
    else
      arm_constraint = arm_constraint.setJointLimits(arm_joints,q(arm_joints,i-1)-ladder_opts.arm_tol,q(arm_joints,i-1)+ladder_opts.arm_tol);
    end
    constraints = [constraints, {arm_constraint}];
  end
%   com_array = repmat(com,1,4);
  for j = 1:2
    pos_eq = ee_info.feet(j).traj.eval(t_data);
    rpy_tol = eval(ee_info.feet(j).rpy_tol_traj,t_data);
    constraints = [ ...
      constraints, ...
      {WorldPositionConstraint(r,ee_info.feet(j).idx, ...
      ee_info.feet(j).pt(1:3), ...
      pos_eq(1:3),pos_eq(1:3))}];
    if foot_supported(j) || ladder_opts.use_swing_foot_euler_constraint
      constraints = [ ...
        constraints, ...
        {WorldEulerConstraint(r,ee_info.feet(j).idx, ...
        pos_eq(4:6)-rpy_tol, ...
        pos_eq(4:6)+rpy_tol)}];
    end
    if hand_grasped(j)
      pos_min = ee_info.hands(j).min_traj.eval(t_data);
      pos_max = ee_info.hands(j).max_traj.eval(t_data);
      pos_eq = (pos_min+pos_max)/2;
      rpy = quat2rpy(pos_eq(4:7));
      pelvis_T_rail = eye(4);
      pelvis_T_rail(1:3,1:3) = roty(30*pi/180);
      pelvis_T_rail(1:3,4) = pos_eq(1:3) - o_T_pelvis(1:3,4);
      o_T_rail = o_T_pelvis*pelvis_T_rail;
      constraints = [ ...
        constraints, ...
        {WorldPositionInFrameConstraint(r, ee_info.hands(j).link_ndx, ...
          ee_info.hands(j).pt, o_T_rail, [-ladder_opts.hand_pos_tol;-ladder_opts.hand_pos_tol;-ladder_opts.hand_pos_tol], [ladder_opts.hand_pos_tol;ladder_opts.hand_pos_tol;ladder_opts.hand_pos_tol]), ...
         WorldGazeOrientConstraint(r,ee_info.hands(j).link_ndx,ee_info.hands(j).axis,pos_min(4:7),ladder_opts.hand_cone_threshold,ladder_opts.hand_threshold)}];
    end
  end

  if i > 1 && ladder_opts.use_incr_com_constraint
    constraints = [ ...
      constraints, ...
      {WorldCoMConstraint(r,com_prev-com_incr_tol_vec,com_prev+com_incr_tol_vec)}];
  end  
  if ladder_opts.use_com_constraint
    com = eval(ladder_opts.comtraj,ts(i));
    com(3) = NaN;
    constraints = [ ...
      constraints, ...
      {WorldCoMConstraint(r,com-com_tol_vec,com+com_tol_vec)}];
  end
  [q(:,i),info,infeasible] = inverseKinPointwise(r,t_data,q_seed,q_nom,constraints{:},ikoptions);
%   if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
  if info > 4 
    if ladder_opts.use_com_constraint
      ladder_opts.com_tol_local = ladder_opts.com_tol;
      while info ~= 1 && ladder_opts.com_tol_local < ladder_opts.com_tol_max
        %display(ladder_opts.com_tol_local(1))
        ladder_opts.com_tol_local = ladder_opts.com_tol_local+0.01;
        constraints{end} = WorldCoMConstraint(r,com-ladder_opts.com_tol_local,com+ladder_opts.com_tol_local);
        [q(:,i),info] = inverseKinPointwise(r,t_data,q_seed,q_nom,constraints{:},ikoptions);
      end
    end
    if info > 4
      disp(infeasible);keyboard
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
  if ladder_opts.compute_intro && i==1
    % Compute plan from q0 to q(:,1)
    nt_init = floor(nt/4);
    dt = mean(diff(ts));
    t_init = 0:dt:nt_init*dt;
    q_init_nom = PPTrajectory(foh([t_init(1),t_init(end)],[q0,q(:,1)]));
    q_init = eval(q_init_nom,t_init);
%     init_constraints = constraints(cellfun(@(con) ~(isa(con,'WorldCoMConstraint')||isa(con,'PostureConstraint')), constraints));
%     [q_init,info] = inverseKinPointwise(r,t_init, ...
%                                         eval(q_init_nom,t_init), ...
%                                         eval(q_init_nom,t_init), ...
%                                         init_constraints{:},ikoptions);
  end
  q_seed = q(:,i);
  constraint_array{i} = constraints;
  kinsol = doKinematics(r,q_seed);
  com_prev = r.getCOM(kinsol);
  com_prev(3) = NaN;
%   q_nom = q(:,i);
  %   disp(info);
end
fprintf('\n');
q_data = q;
t_data = ts;
if ladder_opts.compute_intro
  q_data = [q_init,q];
  t_data = [t_init, ts+t_init(end)+dt];
end
if ladder_opts.use_final_com_constraint
  % Compute plan from q(:,end) to qf
  nt_end = floor(nt/4);
  dt = mean(diff(ts));
  t_end = 0:dt:nt_end*dt;
  t_end_coarse = linspace(t_end(1),t_end(end),3);
%   com_constraint_f = WorldCoMConstraint(r,com,com,t_end(end)*[1,1]);
  com_constraint_f = QuasiStaticConstraint(r);
  com_constraint_f = com_constraint_f.setActive(true);
  com_constraint_f = com_constraint_f.setShrinkFactor(ladder_opts.final_shrink_factor);
  foot1_pts = r.getBodyContacts(ee_info.feet(1).idx);
  foot2_pts = r.getBodyContacts(ee_info.feet(2).idx);
  com_constraint_f = com_constraint_f.addContact(ee_info.feet(1).idx,foot1_pts,ee_info.feet(2).idx,foot2_pts);
  constraints(cellfun(@(con) isa(con,'WorldCoMConstraint'),constraints)) = [];
  [qf,info] = inverseKin(r,q(:,end),qstar,constraints{1:end},com_constraint_f,ikoptions);
  if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); keyboard; end;
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
  t_data = [t_data, t_end + t_data(end) + dt];
end
%x_data = zeros(2*nq,size(q_data,2));
%x_data(1:getNumDOF(r),:) = q_data;
display(err_segments);
if ladder_opts.smooth_output
  for i = 1:nq
    q_data(i,:) = smooth(q_data(i,:)',ladder_opts.smoothing_span);
  end
end
if ladder_opts.n > 1
  t_data = t_data(1:ladder_opts.n:end);
  x_data = zeros(2*nq,length(t_data));
  for i = 1:nq
    x_data(i,:) = decimate(q_data(i,:)',ladder_opts.n);
  end
  q_data = x_data(1:nq,:);
end
end
