function [x_data, t_data] = robotLadderPlanLeanBack(r_minimal,r, q0, qstar, comtraj, link_constraints,support_times,support)

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'robotLadderPlan');
red = {1,0,0};
blue = {0,0,1};
gray = {0.5,0.5,0.5};
black = {0,0,0};
nq = r.getNumDOF();

pelvis = r.findLinkInd('pelvis');
utorso = r.findLinkInd('utorso');
r_foot = r.findLinkInd('r_foot');
l_foot = r.findLinkInd('l_foot');
r_hand = r.findLinkInd('r_hand');
l_hand = r.findLinkInd('l_hand');
neck_joint = findJointIndices(r,'neck');
%ankle_joints = [findJointIndices(r,'r_leg_aky')];
 ankle_joints = [findJointIndices(r,'l_leg_aky'); ...
                 findJointIndices(r,'r_leg_aky')];
% knee_joints = [findJointIndices(r,'l_leg_kny'); ...
%                 findJointIndices(r,'r_leg_kny')];
knee_joints = findJointIndices(r,'l_leg_kny');
arm_joints = findJointIndices(r,'arm');

ladder_opts = struct();
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
  ladder_opts.use_final_com_constraint =    false;
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
  ladder_opts.use_utorso_constraint =       false;
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
  ladder_opts.shrink_factor = 0.5;
end
if ~isfield(ladder_opts,'utorso_threshold') 
  ladder_opts.utorso_threshold = 25*pi/180;
end
if ~isfield(ladder_opts,'pelvis_gaze_threshold') 
  ladder_opts.pelvis_gaze_threshold = 20*pi/180;
end
if ~isfield(ladder_opts,'ankle_limit') 
  ladder_opts.ankle_limit = 15*pi/180*ones(size(ankle_joints));
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
  ladder_opts.arm_tol = 6*pi/180/ladder_opts.n;
end
if ~isfield(ladder_opts,'arm_tol_total') 
  ladder_opts.arm_tol_total = 30*pi/180;
end
com_tol_vec = [ladder_opts.com_tol;ladder_opts.com_tol;NaN];
com_incr_tol_vec = [ladder_opts.com_incr_tol;ladder_opts.com_incr_tol;NaN];

% use_quasistatic_constraint =  true;
% use_com_constraint =          false;
% use_incr_com_constraint =     false;
% use_final_com_constraint =    true;
% use_arm_constraints =         false;
% use_total_arm_constraints =   false;
% use_pelvis_gaze_constraint =  true;
% use_pelvis_constraint =       true;
% use_utorso_constraint =       true;
% use_knee_constraint =         true;
% use_ankle_constraint =        true;
% use_neck_constraint =         true;
% use_collision_constraint =    false;
% use_smoothing_constraint =    false;

% n = 1;
% shrink_factor = 0.1;
% utorso_threshold = 25*pi/180;
% pelvis_gaze_threshold = 20*pi/180;
% ankle_limit = 25*pi/180*ones(size(ankle_joints));
% knee_lb = 30*pi/180*ones(size(knee_joints));
% knee_ub = inf*pi/180*ones(size(knee_joints));
% hand_threshold = sin(1*pi/180);
% hand_cone_threshold = sin(1*pi/180);
% hand_pos_tol = 0.01;
% %pelvis_threshold = 0.08;
% pelvis_threshold = 0.05;
% com_tol = 0.1;
% com_incr_tol = 0.02;
% com_tol_max = 0.5;
% qs_margin = 0.0;
% arm_tol = 6*pi/180/n;
% arm_tol_total = 30*pi/180;
% %comtraj = comtraj + 0.05;
% com_tol_vec = [com_tol;com_tol;NaN];
% com_incr_tol_vec = [com_incr_tol;com_incr_tol;NaN];
% smoothing_tol = 1*pi/180;


% time spacing of samples for IK
tf = comtraj.tspan(2);
dt = max(0.01,tf/1000);
ts = 0:dt:tf;
if length(ts)>1000 % limit numtrueber of IK samples to something reasonable
  ts = linspace(0,tf,100);
  dt = ts(2)-ts(1);
end
nt = length(ts);
%ts(nt+1) = ts(end)+eps;
nt_support = length(support_times);
support_times(end+1) = support_times(end)+eps;
ts = unique([support_times,linspace(0,tf,100)],'sorted');
nt = length(ts);

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
ikoptions = ikoptions.setDebug(true);
ikoptions = ikoptions.setMajorIterationsLimit(5000);
%ikoptions = ikoptions.setSequentialSeedFlag(true);
 %ikoptions = ikoptions.setMex(false);

msg ='Ladder Plan : Computing robot plan...'; disp(msg); send_status(6,0,0,msg);
% logical2str = @(b) regexprep(sprintf('%i',b),{'1','0'},{' ON ',' OFF '});
% fprintf('\n');
% fprintf('Constraint Summary\n');
% fprintf('==================================\n');
% fprintf('Name                      Status    Tolerance\n');
% fprintf('QS Constraint             %s        %4.2f\n', logical2str(ladder_opts.use_quasistatic_constraint),ladder_opts.qs_margin);
% fprintf('Incr. COM Constraint:     %s        %4.2f m\n', logical2str(ladder_opts.use_incr_com_constraint), com_incr_tol);
% fprintf('COM Constraint:           %s        %4.2f m\n', logical2str(ladder_opts.use_com_constraint), com_tol);
% fprintf('Final COM Constraint:     %s\n', logical2str(ladder_opts.use_final_com_constraint));
% fprintf('Pelvis Constraint:        %s        %4.2f m\n', logical2str(ladder_opts.use_pelvis_constraint), pelvis_threshold);
% fprintf('Pelvis GazeConstraint:    %s        %4.2f deg\n', logical2str(ladder_opts.use_pelvis_gaze_constraint),pelvis_gaze_threshold*180/pi);
% fprintf('Torso Constraint:         %s        %4.2f deg\n', logical2str(ladder_opts.use_utorso_constraint),utorso_threshold*180/pi);
% fprintf('Knee Constraint:          %s        [%4.2f deg, %4.2f deg]\n', logical2str(ladder_opts.use_knee_constraint),knee_lb,knee_ub);
% fprintf('Neck Constraint:          %s\n', logical2str(ladder_opts.use_neck_constraint));
% fprintf('Ankle Constraint:         %s\n', logical2str(ladder_opts.use_ankle_constraint));
% fprintf('Arm Constraints (incr):   %s        %4.2f deg\n', logical2str(ladder_opts.use_arm_constraints),arm_tol*180/pi);
% fprintf('Arm Constraints (total):  %s        %4.2f deg\n', logical2str(ladder_opts.use_total_arm_constraints),arm_tol_total*180/pi);

% v = r.constructVisualizer;
% v.display_dt = 0.05;
q = zeros(nq,nt);
q(:,1) = q0;


ee_info.feet(1) = link_constraints([link_constraints.link_ndx] == l_foot);
ee_info.feet(2) = link_constraints([link_constraints.link_ndx] == r_foot);
ee_info.feet(1).idx = l_foot;
ee_info.feet(2).idx = r_foot;
ee_info.hands(1) = link_constraints([link_constraints.link_ndx] == l_hand);
ee_info.hands(2) = link_constraints([link_constraints.link_ndx] == r_hand);
ee_info.hands(1).idx = l_hand;
ee_info.hands(2).idx = r_hand;
% 
% r_hand_constraint = link_constraints([link_constraints.link_ndx] == r_hand);
% r_hand_grasped = ~isempty(r_hand_constraint);
% l_hand_constraint = link_constraints([link_constraints.link_ndx] == l_hand);
% l_hand_grasped = ~isempty(l_hand_constraint);

r_foot_support_data = zeros(size(support_times));
r_foot_support_data(1:end-1) = ...
  cellfun(@(supp) double(any(supp.bodies==r_foot)),support); 
r_foot_support_data(end) = r_foot_support_data(end-1);
r_foot_support_traj = ...
  PPTrajectory(zoh(support_times,r_foot_support_data));
ee_info.feet(2).support_traj = r_foot_support_traj;

l_foot_support_data = zeros(size(support_times));
l_foot_support_data(1:end-1) = ...
  cellfun(@(supp) double(any(supp.bodies==l_foot)),support); 
l_foot_support_data(end) = l_foot_support_data(end-1);
l_foot_support_traj = ...
  PPTrajectory(zoh(support_times,l_foot_support_data));
ee_info.feet(1).support_traj = l_foot_support_traj;

% dup_idx = false(size(support_times));
% for i = 2:nt_support
%   if all([ee_info.feet(1).support_traj.eval(support_times(i-1));ee_info.feet(2).support_traj.eval(support_times(i-1))] ...
%           == [ee_info.feet(1).support_traj.eval(support_times(i));ee_info.feet(2).support_traj.eval(support_times(i))])
%     dup_idx(i) = true;
%   end
% end
% support_times(dup_idx) = [];
% nt_support = length(support_times);

[q_data, t, ee_info] = ladderIK(r,unique([support_times,linspace(0,tf,0)],'sorted'),q0,qstar,ee_info,ladder_opts,ikoptions);

% dup_idx = false(size(support_times));
% i = 2
% while i < size(q_data,2)
%   if max(abs(q_data(:,i)-q_data(:,i-1))) < 5e-2
%     q_data(:,i) = [];
%     support_times(i) = [];
%     t(i) = [];
%   else
%     i = i+1;
%   end
% end
% nt_support = length(support_times);

com_array = zeros(3,nt_support);
for i = 1:nt_support
  kinsol = doKinematics(r,q_data(:,i));
  com_data(:,i) = getCOM(r,kinsol);
end
ladder_opts.comtraj = PPTrajectory(foh(support_times(1:end-1),com_data));
ladder_opts.use_com_constraint = true;
ladder_opts.use_final_com_constraint = true;
ladder_opts.com_tol = 0.01;
ladder_opts.com_tol = 0.01;
ladder_opts.use_swing_foot_euler_constraint = true;
ladder_opts.hand_threshold = sin(5*pi/180);
ladder_opts.compute_intro = true;
ladder_opts.n = 1;
ladder_opts.smooth_output = true;
ladder_opts.smoothing_span = 7;
ladder_opts.smoothing_method = 'moving'; 
% ladder_opts.use_pelvis_constraint = false;
% ladder_opts.use_pelvis_gaze_constraint = false;
% ladder_opts.use_torso_constraint = false;
% ladder_opts.use_incr_com_constraint = true;
[q_data, t_data, ee_info] = ladderIK(r,unique([ts,0:dt:tf],'sorted'),q0,qstar,ee_info,ladder_opts,ikoptions);
x_data = [q_data;zeros(size(q_data))];
% t_data = t;

% n_segments = size(q_data,2)-1;
% t_data = [];
% x_data = [];
% ladder_opts.use_pelvis_constraint = false;
% ladder_opts.com_tol_vec = false;
% ladder_opts.use_pelvis_gaze_constraint = false;
% ladder_opts.use_ankle_constraint = false;
% ladder_opts.use_arm_constraints = false;
% ladder_opts.use_com_constraint = false;
% ladder_opts.use_final_com_constraint = false;
% ladder_opts.use_incr_com_constraint = false;
% ladder_opts.use_knee_constraint = true;
% for i = 1:n_segments
% %   q_seed_traj = PPTrajectory(foh(t(i:i+1),q_data(:,i:i+1)));
% %   q_nom_traj = ConstantTrajectory(qstar);
% %   final_posture_constraint = PostureConstraint(r,t(i+1)*[1,1]);
% %   final_posture_constraint = final_posture_constraint.setJointLimits((1:nq)',q_data(:,i+1),q_data(:,i+1));
% %   [xtraj,info] = inverseKinTraj(r,linspace(t(i),t(i+1),5),q_seed_traj,q_nom_traj,constraint_array{i}{:},final_posture_constraint,ikoptions);
%   [xtraj,info] = ladderIKTraj(r,linspace(t(i),t(i+1),5),q_data(:,i),q_data(:,i+1),qstar,ee_info,ladder_opts,ikoptions);
%   t_data = [t_data,xtraj.tspan(1):dt:xtraj.tspan(2)];
%   x_data = [x_data,eval(xtraj,xtraj.tspan(1):dt:xtraj.tspan(2))];
% end
for i = 1:size(x_data,2)
  kinsol = doKinematics(r,x_data(1:nq,i));
  com = getCOM(r,kinsol);
  com(3) = 0;
  phi = i/size(x_data,2);
  lcmgl.glColor3f(phi,0,1-phi);
  lcmgl.sphere(com,0.02,20,20);
end
lcmgl.switchBuffers();
% basic_constraints = {};
% 
% if use_ankle_constraint
%   ankle_constraint = PostureConstraint(r,tf*[0*0.9,Inf]);
%   ankle_constraint = ankle_constraint.setJointLimits(ankle_joints, ...
%     -ankle_limit, ...
%     10*ankle_limit); 
%   basic_constraints = [basic_constraints, {ankle_constraint}];
% end
% 
% if use_knee_constraint
%   knee_constraint = PostureConstraint(r);
%   knee_constraint = knee_constraint.setJointLimits(knee_joints, ...
%     knee_lb, ...
%     knee_ub);
%  basic_constraints = [basic_constraints, {knee_constraint}];
% end
% 
% if use_neck_constraint
%   neck_constraint = PostureConstraint(r);
%   neck_constraint = neck_constraint.setJointLimits(neck_joint,q0(neck_joint),q0(neck_joint));
%   basic_constraints = [basic_constraints, {neck_constraint}];
% end
% 
% if use_total_arm_constraints %|| length(link_constraints)==2
%   arm_constraint = PostureConstraint(r);
%   arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-arm_tol_total,q0(arm_joints)+arm_tol_total);
%   basic_constraints = [basic_constraints, {arm_constraint}];
% end
% 
% if use_collision_constraint
%   basic_constraints = [ ...
%     basic_constraints
%     {AllBodiesClosestDistanceConstraint(r,0.05,1e3)}];
% end
% 
% kinsol = doKinematics(r,q0);
% if use_utorso_constraint
%   utorso_posquat = forwardKin(r,kinsol,utorso,[0;0;0],2);
%   basic_constraints = [ ...
%     basic_constraints, ...
%     {WorldGazeOrientConstraint(r,utorso,[0;0;1],utorso_posquat(4:7),utorso_threshold,90*pi/180)}];
% end
% 
% if use_pelvis_gaze_constraint
%   basic_constraints = [ ...
%     basic_constraints, ...
%     {WorldGazeOrientConstraint(r,pelvis,[0;0;1],[1;0;0;0],pelvis_gaze_threshold,90*pi/180)}];
% end
% 
% pelvis_xyzrpy = forwardKin(r,kinsol,pelvis,[0;0;0],1);
% o_T_pelvis = HT(pelvis_xyzrpy(1:3),pelvis_xyzrpy(4),pelvis_xyzrpy(5),pelvis_xyzrpy(6));
% o_T_pelvis(1:3,1:3) = eye(3);
% if use_pelvis_constraint
%   basic_constraints = [ ...
%     basic_constraints, ...
%     {WorldPositionInFrameConstraint(r,pelvis, ...
%     [0;0;0], o_T_pelvis, [NaN;-pelvis_threshold;NaN], ...
%     [NaN;pelvis_threshold;NaN])}];
% end
% 
% q_seed = q0;
% q_nom = qstar;
% rpy_tol_max = 30*pi/180;
% rpy_tol_traj = PPTrajectory(foh(linspace(0,tf,5),repmat([0,0,rpy_tol_max,0,0],3,1)));
% for i=1:length(link_constraints)
%   if ~isempty(link_constraints(i).traj)
%     deriv = fnder(link_constraints(i).traj);
%     t_moving = ts(any(eval(deriv,ts) > 0,1));
%     if isempty(t_moving)
%       link_constraints(i).rpy_tol_traj = PPTrajectory(foh([0,tf],repmat([0,0],3,1)));
%     else
%       t_move0 = t_moving(1);
%       t_movef = t_moving(end);
%       link_constraints(i).rpy_tol_traj = PPTrajectory(foh([0,linspace(t_move0,t_movef,3),tf],repmat([0,0,rpy_tol_max,0,0],3,1)));
%     end
%   end
% end
% com_fade_traj = PPTrajectory(foh(linspace(0,tf,4),0.2*[0,1,1,0]));
% 
% msg = '  0%%';
% fprintf(['Progress: ',msg]);
% len_prev_msg = length(sprintf(msg));
% n_err = 0;
% first_err = true;
% err_segments = [];
% [joint_min,joint_max] = getJointLimits(r);
% for i=1:nt
%   t = ts(i);
%   constraints = basic_constraints;
%   
%   r_foot_supported = eval(r_foot_support_traj,t);
%   l_foot_supported = eval(l_foot_support_traj,t);
%   if use_quasistatic_constraint && (r_foot_supported || l_foot_supported)
%     %qsc = QuasiStaticConstraint(r);
%     foot_pts_in_world = [];
%     hand_pts_in_world = [];
%     if r_foot_supported
%       r_foot_pts = r.getBodyContacts(r_foot);
%       r_foot_pos = link_constraints([link_constraints.link_ndx] == r_foot).traj.eval(t);
%       T_r_foot_to_world = [rpy2rotmat(r_foot_pos(4:6)),r_foot_pos(1:3); ... 
%                          zeros(1,3),1];
%       r_foot_pts_in_world = T_r_foot_to_world*[r_foot_pts;ones(1,size(r_foot_pts,2))];
%       foot_pts_in_world = [foot_pts_in_world, r_foot_pts_in_world(1:3,:)];
%       %qsc = qsc.addContact(r_foot,r.getBodyContacts(r_foot));
%     end
%     if l_foot_supported
%       l_foot_pts = r.getBodyContacts(l_foot);
%       l_foot_pos = link_constraints([link_constraints.link_ndx] == l_foot).traj.eval(t);
%       T_l_foot_to_world = [rpy2rotmat(l_foot_pos(4:6)),l_foot_pos(1:3); ... 
%                          zeros(1,3),1];
%       l_foot_pts_in_world = T_l_foot_to_world*[l_foot_pts;ones(1,size(l_foot_pts,2))];
%       foot_pts_in_world = [foot_pts_in_world, l_foot_pts_in_world(1:3,:)];
%       %qsc = qsc.addContact(l_foot,r.getBodyContacts(l_foot));
%     end
%     if r_hand_grasped
%       r_hand_pos = 0.5*(r_hand_constraint.min_traj.eval(t) ...
%                           +r_hand_constraint.min_traj.eval(t));
%       hand_pts_in_world = [hand_pts_in_world, r_hand_pos(1:3,:)];
%     end
%     if l_hand_grasped
%       l_hand_pos = 0.5*(l_hand_constraint.min_traj.eval(t) ...
%                           +l_hand_constraint.min_traj.eval(t));
%       hand_pts_in_world = [hand_pts_in_world, l_hand_pos(1:3,:)];
%     end
%     K = convhulln(foot_pts_in_world(1:2,:)');
%     foot_chull_pts_in_world = foot_pts_in_world(:,K(:,1));
%     foot_chull_pts_in_world(3,:) = min(foot_chull_pts_in_world(3,:));
%     
%     n_foot_chull_pts = size(foot_chull_pts_in_world,2);
%     foot_chull_pts_in_world = foot_chull_pts_in_world*shrink_factor+bsxfun(@times,mean(foot_chull_pts_in_world,2)*(1-shrink_factor),ones(1,n_foot_chull_pts));
%     
%     foot_chull_pts_in_bot = ...
%       o_T_pelvis\[foot_chull_pts_in_world;ones(1,size(foot_chull_pts_in_world,2))];
%     foot_chull_pts_in_bot(3,:) = [];
%     n_chull_pts = size(foot_chull_pts_in_world,2);
%     foot_chull_edges = circshift(foot_chull_pts_in_world,[0,1]) ...
%                        - foot_chull_pts_in_world;
%     foot_chull_edge_directions = ...
%       bsxfun(@rdivide,foot_chull_edges, ...
%              sqrt(sum(foot_chull_edges.^2,1)));
%     if ~isempty(hand_pts_in_world)
%       hand_pts_in_bot = ...
%         o_T_pelvis\[hand_pts_in_world;ones(1,size(hand_pts_in_world,2))];
%       hand_pts_in_bot(3,:) = [];
%       if size(hand_pts_in_world,2)
%         h1_minus_chull_pts = ...
%           bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,1));
%         h1_angles = atan2(h1_minus_chull_pts(2,:),h1_minus_chull_pts(1,:));
%         [~,p1_idx] = min(h1_angles);
%         foot_chull_pts_in_world = ...
%           circshift(foot_chull_pts_in_world,[0 -p1_idx+1]);
%         foot_chull_pts_in_bot = ...
%           circshift(foot_chull_pts_in_bot,[0 -p1_idx+1]);
%         foot_chull_edges = circshift(foot_chull_edges,[0 -p1_idx+1]);
%         foot_chull_edge_directions = ...
%           circshift(foot_chull_edge_directions,[0 -p1_idx+1]);
%         p1_idx = 1;
%         h2_minus_chull_pts = ...
%           bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,2));
%         h2_angles = atan2(h2_minus_chull_pts(2,:),h2_minus_chull_pts(1,:));
%         [~,p2_idx] = max(h2_angles);
% 
%         % Replace edge(p1_idx) with vector from h1 to p1
% %         foot_chull_edge_directions(:,p1_idx) = ...
% %           -normalizeVec(foot_chull_pts_in_world(:,p1_idx) - [hand_pts_in_world(1:2,1); foot_chull_pts_in_world(3,p1_idx)]);
%         foot_chull_edge_directions = ...
%           [foot_chull_edge_directions(:,1), ...
%           -normalizeVec(foot_chull_pts_in_world(:,p1_idx) - [hand_pts_in_world(1:2,1); foot_chull_pts_in_world(3,p1_idx)]),...
%           -normalizeVec([hand_pts_in_world(1:2,2);foot_chull_pts_in_world(3,p2_idx)] - foot_chull_pts_in_world(:,p2_idx)),...
%           foot_chull_edge_directions(:,p2_idx+1:end)];
%         foot_chull_pts_in_world = ...
%           foot_chull_pts_in_world(:,[p1_idx,p1_idx,p2_idx:end]);
% %           foot_chull_pts_in_world(:,[1,p2_idx,p2_idx:end]);
%         p2_idx = 3;
%       elseif size(hand_hold_pts,1)
%         h1_minus_chull_pts = ...
%           bsxfun(@minus,foot_chull_pts_in_bot,hand_pts_in_bot(:,1));
%         h1_angles = atan2(h1_minus_chull_pts(2,:),h1_minus_chull_pts(1,:));
%         [~,p1_idx] = min(h1_angles);
%         foot_chull_pts_in_world = ...
%           circshift(foot_chull_pts_in_world,[0 -p1_idx]);
%         foot_chull_pts_in_bot = ...
%           circshift(foot_chull_pts_in_bot,[0 -p1_idx]);
%         p1_idx = 1;
%       end
%     end
%     for j = 1:size(foot_chull_edge_directions)
%       R = quat2rotmat(quatTransform([1;0;0],foot_chull_edge_directions(:,j)));
%       P = foot_chull_pts_in_world(:,j);
%       T = [R,P;zeros(1,3),1];
%       com_halfspace_constraint = ...
%         WorldCoMInFrameConstraint(r,T,[NaN;qs_margin;NaN],[NaN;NaN;NaN]);
%       constraints = [constraints, {com_halfspace_constraint}];
%     end
% 
%     % Draw all support points
%     lcmgl.glColor3f(gray{:});
%     for pt = foot_pts_in_world
%       lcmgl.sphere(pt,0.01,20,20);
%     end
%     % Draw convex hull points
%     lcmgl.glColor3f(blue{:});
%     for pt = foot_chull_pts_in_world
%       lcmgl.sphere(pt,0.02,20,20);
%     end
%     % Draw edges
%     %lcmgl.glColor3f(black{:});
%     %lcmgl.glLineWidth(4);
%     %for j = 1:n_chull_pts
%       %lcmgl.glBegin(lcmgl.LCMGL_LINES);
%       %lcmgl.glVertex3d(foot_chull_pts_in_world(1,j),foot_chull_pts_in_world(2,j),foot_chull_pts_in_world(3,j))
%       %lcmgl.glVertex3d(foot_chull_pts_in_world(1,j)+foot_chull_edges(1,j),foot_chull_pts_in_world(2,j)+foot_chull_edges(2,j),foot_chull_pts_in_world(3,j)+foot_chull_edges(3,j))
%       %lcmgl.glEnd()
%     %end
%     % Draw hand points
%     lcmgl.glColor3f(red{:});
%     for pt = hand_pts_in_world
%       pt(3) = 0;
%       lcmgl.sphere(pt,0.02,20,20);
%     end
%     lcmgl.glColor3f(black{:});
%     lcmgl.sphere(foot_chull_pts_in_world(:,p1_idx),0.03,20,20);
%     lcmgl.glColor3f(gray{:});
%     lcmgl.sphere(foot_chull_pts_in_world(:,p2_idx),0.03,20,20);
%     lcmgl.glLineWidth(2);
%     for j = 1:size(foot_chull_edge_directions,2)
%       aa = quat2axis(quatTransform([1;0;0],foot_chull_edge_directions(:,j)));
%       lcmgl.glTranslated(foot_chull_pts_in_world(1,j), ...
%                         foot_chull_pts_in_world(2,j), ...
%                         foot_chull_pts_in_world(3,j));
%       lcmgl.glRotated(-aa(4)*180/pi,aa(1),aa(2),aa(3));
%       lcmgl.glDrawAxes();
%       lcmgl.glRotated(aa(4)*180/pi,aa(1),aa(2),aa(3));
%       lcmgl.glTranslated(-foot_chull_pts_in_world(1,j), ...
%                         -foot_chull_pts_in_world(2,j), ...
%                         -foot_chull_pts_in_world(3,j));
%     end
%     lcmgl.switchBuffers();
% 
%     %qsc = qsc.setShrinkFactor(shrink_factor);
%     %qsc = qsc.setActive(true);
%     %constraints = [constraints, {qsc}];
%   end
%   if use_arm_constraints
%     arm_constraint = PostureConstraint(r);
%     if i==1
%       arm_constraint = arm_constraint.setJointLimits(arm_joints,q0(arm_joints)-arm_tol,q0(arm_joints)+arm_tol);
%     else
%       arm_constraint = arm_constraint.setJointLimits(arm_joints,q(arm_joints,i-1)-arm_tol,q(arm_joints,i-1)+arm_tol);
%     end
%     constraints = [constraints, {arm_constraint}];
%   end
% 
%   if use_smoothing_constraint
%     smoothing_constraint = PostureConstraint(r);
%     if i==1
%       smoothing_constraint = smoothing_constraint.setJointLimits((1:nq)', ...
%         max(q0-smoothing_tol,joint_min), ...
%         min(q0+smoothing_tol,joint_max-1e-6));
%     else
%       smoothing_constraint = smoothing_constraint.setJointLimits(1:nq, ...
%         max(q(:,i-1)-smoothing_tol,joint_min), ...
%         min(q(:,i-1)+smoothing_tol,joint_max-1e-6));
%     end
%     constraints = [basic_constraints, {smoothing_constraint}];
%   end
%   com = eval(comtraj,ts(i));
%   com(3) = NaN;
% %   com_array = repmat(com,1,4);
%   for j = 1:length(link_constraints)
%     if ~isempty(link_constraints(j).traj)
%       pos_eq = link_constraints(j).traj.eval(t);
%       rpy_tol = eval(link_constraints(j).rpy_tol_traj,t);
%       constraints = [ ...
%         constraints, ...
%         {WorldPositionConstraint(r,link_constraints(j).link_ndx, ...
%             link_constraints(j).pt(1:3), ...
%             pos_eq(1:3),pos_eq(1:3)),...
%          WorldEulerConstraint(r,link_constraints(j).link_ndx, ...
%             pos_eq(4:6)-rpy_tol, ...
%             pos_eq(4:6)+rpy_tol)}];
%     else
%       %pos_min = link_constraints(j).min_traj.eval(t)-0.1;
%       %pos_max = link_constraints(j).max_traj.eval(t)+0.1;
%       pos_min = link_constraints(j).min_traj.eval(t);
%       pos_max = link_constraints(j).max_traj.eval(t);
%       pos_eq = (pos_min+pos_max)/2;
%       rpy = quat2rpy(pos_eq(4:7));
%       pelvis_T_rail = eye(4);
%       pelvis_T_rail(1:3,1:3) = roty(30*pi/180);
%       pelvis_T_rail(1:3,4) = pos_eq(1:3) - o_T_pelvis(1:3,4);
%       o_T_rail = o_T_pelvis*pelvis_T_rail;
%       constraints = [ ...
%         constraints, ...
%         {WorldPositionInFrameConstraint(r, link_constraints(j).link_ndx, ...
%           link_constraints(j).pt, o_T_rail, [-hand_pos_tol;-hand_pos_tol;-hand_pos_tol], [hand_pos_tol;hand_pos_tol;hand_pos_tol]), ...
%          WorldGazeOrientConstraint(r,link_constraints(j).link_ndx,link_constraints(j).axis,pos_min(4:7),hand_cone_threshold,hand_threshold)}];
%          %WorldQuatConstraint(r,link_constraints(j).link_ndx,pos_min(4:7),hand_threshold)}];
% %       com_array(1:2,j) = (pos_min(1:2)+pos_max(1:2))/2;
%     end
%   end
% %   com = eval(com_fade_traj,t)*mean(com_array,2) + (1-eval(com_fade_traj,t))*com;
% 
%   if i > 1 && use_incr_com_constraint
%     constraints = [ ...
%       constraints, ...
%       {WorldCoMConstraint(r,com_prev-com_incr_tol_vec,com_prev+com_incr_tol_vec)}];
%   end  
%   if use_com_constraint
%     constraints = [ ...
%       constraints, ...
%       {WorldCoMConstraint(r,com-com_tol_vec,com+com_tol_vec)}];
%   end
%   [q(:,i),info,infeasible] = inverseKinPointwise(r,t,q_seed,q_nom,constraints{:},ikoptions);
% %   if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
%   if info > 4 
%     if use_com_constraint
%       com_tol_local = com_tol;
%       while info ~= 1 && com_tol_local < com_tol_max
%         %display(com_tol_local(1))
%         com_tol_local = com_tol_local+0.01;
%         constraints{end} = WorldCoMConstraint(r,com-com_tol_local,com+com_tol_local);
%         [q(:,i),info] = inverseKinPointwise(r,t,q_seed,q_nom,constraints{:},ikoptions);
%       end
%     end
%     if info > 4
%       disp(infeasible);keyboard
%       n_err = n_err+1; 
%       if first_err
%         err_segments(end+1,1) = i/nt;
%         first_err = false;
%       else
%         err_segments(end,2) = i/nt;
%       end
%     else
%       first_err = true;
%     end
%   else
%     first_err = true;
%   end;
%   msg = '%3.0f%% (No. Errors: %4.0f)';
%   fprintf([repmat('\b',1,len_prev_msg), msg],i/nt*100,n_err);
%   len_prev_msg = length(sprintf(msg,i/nt*100,n_err));
%   if i==1
%     % Compute plan from q0 to q(:,1)
%     nt_init = floor(nt/4);
%     t_init = 0:dt:nt_init*dt;
%     q_init_nom = PPTrajectory(foh([t_init(1),t_init(end)],[q0,q(:,1)]));
%     [q_init,info] = inverseKinPointwise(r,t_init, ...
%                                         eval(q_init_nom,t_init), ...
%                                         eval(q_init_nom,t_init), ...
%                                         constraints{:},ikoptions);
%   end
%   q_seed = q(:,i);
%   kinsol = doKinematics(r,q_seed);
%   com_prev = r.getCOM(kinsol);
%   com_prev(3) = NaN;
% %   q_nom = q(:,i);
%   %   disp(info);
% end
% fprintf('\n');
% % q_data = q;
% % t = ts;
% q_data = [q_init,q];
% t = [t_init, ts+t_init(end)+dt];
% if use_final_com_constraint
%   % Compute plan from q(:,end) to qf
%   nt_end = floor(nt/4);
%   t_end = 0:dt:nt_init*dt;
%   t_end_coarse = linspace(t_end(1),t_end(end),3);
% %   com_constraint_f = WorldCoMConstraint(r,com,com,t_end(end)*[1,1]);
%   com_constraint_f = QuasiStaticConstraint(r);
%   com_constraint_f = com_constraint_f.setActive(true);
%   com_constraint_f = com_constraint_f.setShrinkFactor(0.5);
%   foot1_pts = r.getBodyContacts(link_constraints(1).link_ndx);
%   foot2_pts = r.getBodyContacts(link_constraints(2).link_ndx);
%   com_constraint_f = com_constraint_f.addContact(link_constraints(1).link_ndx,foot1_pts,link_constraints(2).link_ndx,foot2_pts);
%   constraints(cellfun(@(con) isa(con,'WorldCoMConstraint'),constraints)) = [];
%   [qf,info] = inverseKin(r,qstar,qstar,constraints{1:end},com_constraint_f,ikoptions);
%   if info ~= 1, warning('robotLaderPlanner:badInfo','info = %d',info); end;
%   q_end_nom = PPTrajectory(foh([t_end(1),t_end(end)],[q(:,end),qf]));
%   end_posture_constraint = PostureConstraint(r,t_end_coarse(end)*[1,1]);
%   end_posture_constraint = end_posture_constraint.setJointLimits((1:nq)',qf,qf);
%   % [q_end,info] = inverseKinPointwise(r,t_end, ...
%   %   eval(q_end_nom,t_end), ...
%   %   eval(q_end_nom,t_end), ...
%   %   constraints{:},ikoptions);
%   [x_end_traj,info] = inverseKinTraj(r,t_end_coarse, ...
%     q_end_nom, ...
%     q_end_nom, ...
%     constraints{:},end_posture_constraint,ikoptions);
%   x_end = eval(x_end_traj,t_end);
%   q_end = x_end(1:nq,:);
%   % q_end=[];
%   % [q(:,2:end),info] = inverseKinPointwise(r,ts(2:end),repmat(q0,1,nt-1),repmat(qstar,1,nt-1),constraints{:},ikoptions);
%   % q = q(:,1:5:end);
%   q_data = [q_data,q_end];
%   t = [t, t_end + t(end) + dt];
% end
% for i = 1:size(q_data,2)
%   kinsol = doKinematics(r,q_data(:,i));
%   com = getCOM(r,kinsol);
%   com(3) = 0;
%   phi = i/size(q_data,2);
%   lcmgl.glColor3f(phi,0,1-phi);
%   lcmgl.sphere(com,0.02,20,20);
% end
% lcmgl.switchBuffers();
% %x_data = zeros(2*nq,size(q_data,2));
% %x_data(1:getNumDOF(r),:) = q_data;
% display(err_segments);
% t = t(1:ladder_opts.n:end);
% x_data = zeros(2*nq,length(t));
% for i = 1:nq
%   x_data(i,:) = decimate(q_data(i,:)',ladder_opts.n);
% end
%ts = ts(1:end-1)
end
