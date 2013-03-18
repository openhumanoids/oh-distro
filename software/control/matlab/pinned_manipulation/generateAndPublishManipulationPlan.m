function generateAndPublishManipulationPlan(r,plan_pub,x0,rep_goal,lep_goal)

  disp('Generating plan...');

  % generate robot plan
  T = 5.0; % seconds, hard coded for now
  dt = 1;
  ts = 0:dt:T; % plan timesteps

  q0 = x0(1:getNumDOF(r));

  % get foot positions
  kinsol = doKinematics(r,q0);
    
  rfoot_body = r.findLink('r_foot');
  lfoot_body = r.findLink('l_foot');

%   rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0],true);
%   lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0],true);
  rfoot0 = forwardKin(r,kinsol,rfoot_body,...
      rfoot_body.contact_pts(:,[rfoot_body.collision_group{1}]),true); 
  rfoot0 = mean(rfoot0,2);
  
  lfoot0 = forwardKin(r,kinsol,lfoot_body,...
      lfoot_body.contact_pts(:,[lfoot_body.collision_group{1}]),true);
  lfoot0 = mean(lfoot0,2);
  
  % compute fixed COM goal
  gc = contactPositions(r,q0);
  k = convhull(gc(1:2,:)');
  com0 = getCOM(r,q0);
  comgoal = [mean(gc(1:2,k),2);com0(3)];
  comgoal = com0; % DOnt move com for now as this is pinned manipulation
  
  r_hand_body = findLink(r,'r_hand');
  l_hand_body = findLink(r,'l_hand');
  
  
  % compute EE trajectories

  rhand0 = forwardKin(r,kinsol,r_hand_body,[0;0;0],true);
  lhand0 = forwardKin(r,kinsol,l_hand_body,[0;0;0],true);
 
  T_world_hand_r.p = [rhand0(1);rhand0(2);rhand0(3)];
  T_world_hand_r.M = angle2dcm(rhand0(4),rhand0(5),rhand0(6),'XYZ');
  T_world_hand_l.p = [lhand0(1);lhand0(2);lhand0(3)];
  T_world_hand_l.M = angle2dcm(lhand0(4),lhand0(5),rhand0(6),'XYZ');
  
  % Goals are presented in palm frame, must be transformed to hand coordinate frame
  % Using notation similar to KDL.
  % fixed transform between hand and palm as specified in the urdf
  T_hand_palm_l.p = [0;0.1;0];
  T_hand_palm_l.M = angle2dcm(1.57079,-0,1.57079, 'ZYX');
  T_hand_palm_r.p = [0;-0.1;0];
  T_hand_palm_r.M = angle2dcm(-1.57079,-0,-1.57079, 'ZYX');
  
  T_hand_palm_l.p = [0;0.0;0];
  T_hand_palm_l.M = angle2dcm(0,0,0, 'ZYX');
  T_hand_palm_r.p = [0;0.0;0];
  T_hand_palm_r.M = angle2dcm(0,0,0, 'ZYX');

% the inverse transformation
  T_palm_hand_l.p = -T_hand_palm_l.p;
  T_palm_hand_l.M = inv(T_hand_palm_l.M);
  T_palm_hand_r.p = -T_hand_palm_r.p;
  T_palm_hand_r.M = inv(T_hand_palm_r.M);   
   
  
  %Tr and Tl need to be
  Tr = zeros(4);
  Tr(1:3,1:3) = T_hand_palm_r.M;
  Tr(1:4,4) = [T_hand_palm_r.p(:); 1];
  Tl = zeros(4);
  Tl(1:3,1:3) = T_hand_palm_l.M;
  Tl(1:4,4) = [T_hand_palm_l.p(:); 1];
  
  if(isempty(rep_goal))
      rep_goal = [1;forwardKin(r,kinsol,r_hand_body,[0;0;0],true)];
      rhandT  = rep_goal(2:7);
  else
      rhandT = zeros(6,1);
      % Desired position of palm in world frame
      T_world_palm_r.p = rep_goal(2:4);
      T_world_palm_r.M = angle2dcm(rep_goal(5),rep_goal(6),rep_goal(7),'XYZ');
      T_world_hand_r.p = T_world_palm_r.p + T_palm_hand_r.p;
      T_world_hand_r.M = T_world_palm_r.M*T_palm_hand_r.M;
      rhandT(1:3) = T_world_hand_r.p;
      [rhandT(4),rhandT(5),rhandT(6)]=dcm2angle(T_world_hand_r.M,'XYZ');
  end
  
  if(isempty(lep_goal))
      lep_goal = [1;forwardKin(r,kinsol,l_hand_body,[0;0;0],true)];
      lhandT  = lep_goal(2:7);
  else
      lhandT = zeros(6,1);
      % Desired position of palm in world frame
      T_world_palm_l.p = lep_goal(2:4);
      T_world_palm_l.M = angle2dcm(lep_goal(5),lep_goal(6),lep_goal(7),'XYZ');
      T_world_hand_l.p = T_world_palm_l.p + T_palm_hand_l.p;
      T_world_hand_l.M = T_world_palm_l.M*T_palm_hand_l.M;
      lhandT(1:3) = T_world_hand_l.p;
      [lhandT(4),lhandT(5),lhandT(6)]=dcm2angle(T_world_hand_l.M,'XYZ');
  end
 

% rhandT = rpalmT;
% lhandT = lpalmT;

  % ignore orietnation
  rhand0 = rhand0(1:3);
  lhand0 = lhand0(1:3);
  rhandT = rhandT(1:3);
  lhandT = lhandT(1:3);

  r_hand_pos = PPTrajectory(foh([0,T],[rhand0,rhandT]));
  l_hand_pos = PPTrajectory(foh([0,T],[lhand0,lhandT]));

  ind = getActuatedJoints(r);
  cost = Point(r.getStateFrame,1);
  cost.pelvis_x = 100;
  cost.pelvis_y = 100;
  cost.pelvis_z = 100;
  cost.pelvis_roll = 1000;
  cost.pelvis_pitch = 1000;
  cost.pelvis_yaw = 0;
  cost.back_mby = 100;
  cost.back_ubx = 100;
  cost = double(cost);
  ikoptions = struct();
  ikoptions.Q = diag(cost(1:getNumDOF(r)));
  ikoptions.q_nom = q0;

  %v = r.constructVisualizer();

  q = q0;
  q_d = q(ind);
  for i=2:length(ts)
    t = ts(i);
    tic;
%    q(:,i) = inverseKin(r,q(:,i-1),0,comgoal,rfoot_body,'heel',rfoot0, ...
%        lfoot_body,'heel',lfoot0,r_hand_body,'default',r_hand_pos.eval(t), ...
%        l_hand_body,'default',l_hand_pos.eval(t),ikoptions);
    q(:,i) = inverseKin(r,q(:,i-1),0,[nan;nan;nan],rfoot_body,'heel',rfoot0, ...
        lfoot_body,'heel',lfoot0,r_hand_body,'default',r_hand_pos.eval(t), ...
        l_hand_body,'default',l_hand_pos.eval(t),ikoptions);    
    toc;

    
%     q(:,i) = inverseKin(r,q(:,i-1),0,comgoal,...
%         r_hand_body,'default',r_hand_pos.eval(t), ...
%         l_hand_body,'default',l_hand_pos.eval(t),ikoptions);
    q_d(:,i) = q(ind,i);
   % v.draw(t,q(:,i));
  end
  qd_frame = AtlasPositionRef(r);
  des_traj = setOutputFrame(PPTrajectory(spline(ts,q_d)),qd_frame);

  % publish robot plan
  disp('Publishing plan...');
  xtraj = zeros(getNumStates(r),length(ts));
  xtraj(1:getNumDOF(r),:) = q;
%  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
%  joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
%  plan_pub = RobotPlanPublisher('atlas',joint_names,true,'CANDIDATE_ROBOT_PLAN');
%  plan_pub.publish(ts,xtraj,des_traj);
  plan_pub.publish(ts,xtraj);
end
