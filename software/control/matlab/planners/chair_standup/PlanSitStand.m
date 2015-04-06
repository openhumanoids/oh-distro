classdef PlanSitStand
  
  methods (Static)
    
    function [qtraj,supports,support_times] = plan(r,x0,plan_type,options)
      
      if ~any(strcmp(plan_type,{'sit','stand','squat','stand_from_squat'}))
        error('DRC:PLANSITSTAND','plan type must be one of {sit,stand,squat,stand_from_squat}');
      end
      
      if nargin < 4
        options = struct();
      end
      
      if ~isfield(options,'chair_height')
        chair_height = 1/2;
      else
        chair_height = options.chair_height;
      end
      
      if ~isfield(options,'speed')
        speed = 1;
      else
        speed = options.speed;
      end
      
      nq = r.getNumPositions();
      np = nq;
      q0 = x0(1:nq);
      
      %% Add in the appropriate path
      path = [getenv('DRC_BASE'),'/software/control/matlab/planners/prone'];
      addpath(path);
      
      % this is where q_sol gets loaded, we actually use this
      load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_standup_data_new.mat']);

      failed_constraint_flag = 0;      
      
      % robot = robot.removeCollisionGroupsExcept({});
      kpt = KinematicPoseTrajectory(r,{});
      kpt = kpt.useRobotiqHands();
      kpt = kpt.addTerrain();
      [~,kpt] = kpt.addCollisionGeometryToRobot();
      % kpt = kpt.setConstraintTol(0.03);
      % kpt = kpt.setConstraintErrTol(0.03);
      
      
      
      %% Torque Constraint
      joint_names = kpt.robot.getPositionFrame.coordinates;
      idx_arm = ~cellfun('isempty',strfind(joint_names,'arm'));
      idx_back = ~cellfun('isempty',strfind(joint_names,'back'));
      % idx = or(idx_arm,idx_back);
      idx = or(idx_arm,idx_back);
      names_arm_back = joint_names(idx);
      torque_multiplier = 1;
      torque_multiplier_back = 1;
      pmin = Point(kpt.robot.getInputFrame,r.umin);
      pmax = Point(kpt.robot.getInputFrame,r.umax);
      
      
      lb = zeros(length(names_arm_back),1);
      ub = lb;
      joint_idx = zeros(length(names_arm_back),1);
      
      
      for j = 1:length(joint_idx)
        name = names_arm_back{j};
        joint_idx(j) = kpt.robot.findPositionIndices(name);
        lb(j) = pmin.([name,'_motor'])*torque_multiplier;
        ub(j) = pmax.([name,'_motor'])*torque_multiplier;
        if strfind(name,'back')
          lb(j) = pmin.([name,'_motor'])*torque_multiplier_back;
          ub(j) = pmax.([name,'_motor'])*torque_multiplier_back;
        end
      end
      
      torque_constraint = GravityCompensationTorqueConstraint(kpt.robot,joint_idx,lb,ub);
      
      %% Hands above ground constraint
      kinsol = r.doKinematics(q0);
      floor_height = r.forwardKin(kinsol,r.findLinkId('r_foot'),[0;0;0]);
      floor_height = floor_height(3);
      min_hand_height = floor_height + 0.05;
      
      lb = [nan;nan;min_hand_height];
      ub = [nan;nan;nan];
      lb = repmat(lb,1,size(kpt.c('l_hand'),2));
      ub = repmat(ub,1,size(kpt.c('l_hand'),2));
      l_hand_above_ground = WorldPositionConstraint(kpt.robot, kpt.linkId('l_hand'),kpt.c('l_hand'),lb,ub);
      r_hand_above_ground = WorldPositionConstraint(kpt.robot, kpt.linkId('r_hand'),kpt.c('r_hand'),lb,ub);
      
      %% Setup
      atlas_fp = load([getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v4_fp.mat']);
      xstar = atlas_fp.xstar;
      qstar = xstar(1:nq);
      arm_idx = kpt.robot.findPositionIndices('arm');
      
      kinsol = r.doKinematics(q0);
      l_foot_pos = r.forwardKin(kinsol,kpt.linkId('l_foot'),kpt.c('l_foot'));
      ground_height = l_foot_pos(3,1);
      pelvis_height = chair_height + ground_height;
      
      %% Pelvis height constraints
      lb = [nan;nan;pelvis_height];
      ub = [nan;nan;pelvis_height];
      lb = repmat(lb,1,size(kpt.c('l_fpelvis'),2));
      ub = repmat(ub,1,size(kpt.c('l_fpelvis'),2));
      
      l_pelvis_height = WorldPositionConstraint(kpt.robot,kpt.linkId('l_fpelvis'),kpt.c('l_fpelvis'),lb,ub);
      r_pelvis_height = WorldPositionConstraint(kpt.robot,kpt.linkId('r_fpelvis'),kpt.c('r_fpelvis'),lb,ub);

      % add an m_pelvis point

      %% Pelvis gaze constraint
      pelvis_gaze_constraint = WorldGazeDirConstraint(kpt.robot,kpt.robot.findLinkId('utorso'),[0;0;1],[0;0;1],pi/2);
      
      %% Weighting matrix
      Q = eye(nq);
      Q(1:6,1:6) = zeros(6,6);
      back_x_idx = r.findPositionIndices('back_bkx');
      back_z_idx = r.findPositionIndices('back_bkz');
      back_y_idx = r.findPositionIndices('back_bky');
      back_idx = r.findPositionIndices('back_idx');
      back_xz_idx = [back_z_idx,back_x_idx];
      % Q(back_xz_idx,back_xz_idx) = 20*eye(2);
      % Q(4:5,4:5) = 10*eye(2);
      
      %% Nominal standing pose
      atlas_fp = load([getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v4_fp.mat']);
      qstar = atlas_fp.xstar(1:nq);

      
      if strcmp(plan_type,'sit') || strcmp(plan_type, 'squat')
        
        
        %% Sitting normally
        contacts = {'l_foot','r_foot','r_fpelvis','l_fpelvis'};
        options.constraints = {torque_constraint,l_pelvis_height,r_pelvis_height,r_hand_above_ground,l_hand_above_ground,pelvis_gaze_constraint};
        
        options.enforce_collision = 0;
        options.enforce_contact = 0;
        options.enforce_quasistatic = 1;
        kpt.shrink_factor = 0.5;
        options.no_movement.contacts = {'l_foot','r_foot'};
        options.no_movement.q = q0;
        options.Q = Q;
        
        q_nom = q_sol(:,1);
        
        % rotate to align it with the current position
        q_nom(1:2) = q0(1:2);
        q_nom(4:6) = q0(4:6);
        q_nom(back_idx) = qstar(back_idx);
        
        options.qs_contacts = {'l_foot','r_foot','l_fpelvis','r_fpelvis'};
        [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_2 = q;
        
        %% Sitting with COM over feet
        clear options;
        contacts = {'l_foot','r_foot','r_fpelvis','l_fpelvis'};
        options.constraints = {torque_constraint,r_hand_above_ground,l_hand_above_ground,pelvis_gaze_constraint};
        
        options.enforce_collision = 0;
        options.enforce_contact = 0;
        options.enforce_quasistatic = 1;
        kpt.shrink_factor = 0.5;
        options.no_movement.contacts = {'l_foot','r_foot','l_fpelvis','r_fpelvis'};
        options.no_movement.q = q_2;
        
        q_nom = q_sol(:,2);
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        q_nom(back_idx) = qstar(back_idx);
        options.Q = Q;
        options.qs_contacts = {'l_foot','r_foot'};
        [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_1 = q;
        
        %$ Construct and rescale the trajectories
        max_degrees_per_second = 15;
        max_base_meters_per_second = 0.05;
        joint_v_max = repmat(max_degrees_per_second*pi/180, r.getNumVelocities()-3, 1);
        joint_v_max = speed*3/2*joint_v_max;
        xyz_v_max = repmat(max_base_meters_per_second,3,1);
        qd_max = [xyz_v_max;joint_v_max];
        
        qtraj_1 = kpt.constructTrajectory([q0,q_1]);
        qtraj_2 = kpt.constructTrajectory([q_1,q_2]);
        
        qtraj_1 = rescalePlanTiming(qtraj_1,qd_max);
        qtraj_2 = rescalePlanTiming(qtraj_2,qd_max);
        qtraj_1_smooth = PlanSitStand.touchUpTrajectory(r,qtraj_1);
        qtraj_2_smooth = PlanSitStand.touchUpTrajectory(r,qtraj_2);
        qtraj_2_smooth = qtraj_2_smooth.shiftTime(qtraj_1_smooth.tspan(2));
        qtraj = qtraj_1_smooth.append(qtraj_2_smooth);
        
        t0 = qtraj_1_smooth.tspan(1);
        t1 = qtraj_1_smooth.tspan(2);
        tf = qtraj.tspan(2);
        support_times = [t0,t1,tf];
        
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
        supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
        
        supports(2).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),r.findLinkId('pelvis'),r.findLinkId('pelvis')];
        supports(2).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot'),kpt.c('l_fpelvis'),kpt.c('r_fpelvis')};
        
        supports(3) = supports(2);
        
        %% Return a squating plan rather than a full sitdown plan
        if strcmp(plan_type,'squat')
          qtraj = qtraj_1_smooth;
          support_times = [t0,t1];
          supports = struct('bodies',{},'contact_pts',{});
          supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
          supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
          supports(2) = supports(1);
        end
        
      end
      
      if strcmp(plan_type,'stand') || strcmp(plan_type,'stand_from_squat')
        %% Sitting with COM over feet
        clear options;
        contacts = {'l_foot','r_foot','r_fpelvis','l_fpelvis'};
        options.constraints = {torque_constraint,l_hand_above_ground,r_hand_above_ground,pelvis_gaze_constraint};
        
        options.enforce_collision = 0;
        options.enforce_contact = 0;
        options.enforce_quasistatic = 1;
        kpt.shrink_factor = 0.5;
        options.no_movement.contacts = contacts;
        options.no_movement.q = q0;
        kpt.shrink_factor = 0.5;
        
        q_nom = q_sol(:,2);
        q_nom(1:6) = q0(1:6);
        q_nom(back_idx) = qstar(back_idx);
        options.Q = Q;
        
        options.qs_contacts = {'l_foot','r_foot'};
        [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_1 = q;
        
        %% Standing in nominal pose
        clear options;
        disp('solving for standing pose')
        contacts = {'l_foot','r_foot'};
        options.no_movement.contacts = contacts;
        options.no_movement.q = q0;
        options.enforce_collision = 0;
        options.enforce_contacts = 0;
        options.enforce_quasistatic = 1;
        options.qs_contacts = {'l_foot','r_foot'};
        options.Q = Q;
        q_nom = qstar;
        q_nom(1:2) = q0(1:2);
        kpt = kpt.setConstraintTol(0.01);
        [q,F,info,infeasible_constraint,ik] = kpt.findPose(contacts,{},q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_2 = q;
        
        
        % now we want to rescale the trajectory
        max_degrees_per_second = 15;
        max_base_meters_per_second = 0.05;
        joint_v_max = repmat(max_degrees_per_second*pi/180, r.getNumVelocities()-3, 1);
        joint_v_max = speed*3/2*joint_v_max;
        xyz_v_max = repmat(max_base_meters_per_second,3,1);
        qd_max = [xyz_v_max;joint_v_max];
        
        qtraj_1 = kpt.constructTrajectory([q0,q_1]);
        qtraj_2 = kpt.constructTrajectory([q_1,q_2]);
        
        qtraj_1 = rescalePlanTiming(qtraj_1,qd_max);
        qtraj_2 = rescalePlanTiming(qtraj_2,qd_max);
        qtraj_1_smooth = PlanSitStand.touchUpTrajectory(r,qtraj_1);
        qtraj_2_smooth = PlanSitStand.touchUpTrajectory(r,qtraj_2);
        qtraj_2_smooth = qtraj_2_smooth.shiftTime(qtraj_1_smooth.tspan(2));
        qtraj = qtraj_1_smooth.append(qtraj_2_smooth);
        
        t0 = qtraj_1_smooth.tspan(1);
        t1 = qtraj_1_smooth.tspan(2);
        tf = qtraj.tspan(2);
        support_times = [t0,t1,tf];
        
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),r.findLinkId('pelvis'),r.findLinkId('pelvis')];
        supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot'),kpt.c('l_fpelvis'),kpt.c('r_fpelvis')};
        
        supports(2).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
        supports(2).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
        
        supports(3) = supports(2);
        
        % just use q_2, which is the standing pose that we just found
        if strcmp(plan_type,'stand_from_squat')
          qtraj = kpt.constructTrajectory([q0,q_2]);
          qtraj = PlanSitStand.touchUpTrajectory(r,qtraj);
          qtraj = rescalePlanTiming(qtraj,qd_max);
          t0 = qtraj.tspan(1);
          t1 = qtraj.tspan(2);
          supports = struct('bodies',{},'contact_pts',{});
          supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
          supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};

          supports(2) = supports(1);
          support_times = [t0,t1];
        end
      end

      if failed_constraint_flag
        disp('NOT ALL CONSTRAINTS SATISFIED');
      else
        disp('all constraints satisfied');
      end                  
    end
    
    function qtraj_new = touchUpTrajectory(r,qtraj)
      t0 = qtraj.tspan(1);
      tf = qtraj.tspan(2);
      
      q0 = qtraj.eval(t0);
      ts = linspace(t0,tf,20);
      q_seed = qtraj.eval(ts);
      kinsol = r.doKinematics(q0);
      pts = [1,-1,0; 0, 0, 1 ; 0,0,0];
      
      
      r_foot_position = r.forwardKin(kinsol,r.findLinkId('r_foot'),pts);
      l_foot_position = r.forwardKin(kinsol,r.findLinkId('l_foot'),pts);
      
      r_foot_constraint = WorldPositionConstraint(r,r.findLinkId('r_foot'),pts,r_foot_position,r_foot_position);
      l_foot_constraint = WorldPositionConstraint(r,r.findLinkId('l_foot'),pts,l_foot_position,l_foot_position);
      constraints = {l_foot_constraint,r_foot_constraint};
      
      [q_sol,info,infeasible_constraint] = r.inverseKinPointwise(ts,q_seed,q_seed,constraints{:});
      qtraj_new = PPTrajectory(pchip(ts,q_sol));
    end
    
  end
end













