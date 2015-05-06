classdef PlanSitStand


  properties
    plan_options
    back_gaze_constraint
    back_gaze_constraint_tight
    back_vertical_gaze_constraint
    pelvis_gaze_constraint
    torque_constraint
    pelvis_contacts
    shoulder_in_constraints
    back_z_constraint
    back_z_constraint_soft
    back_y_constraint
    hand_up_tuck_constraint
    arm_nominal_posture_constraint
    elbow_ind
    elbow_angles
    r
    kpt
    xstar
    qstar
    nq
    Q
    q_sol
    qd_max;
    pelvis_bodies
    pelvis_contact_pts
    back_idx
    back_bkz_idx
    arm_idx
    min_distance_constraint
    handle
    pose_data
    rpp
  end

  methods
    function obj = PlanSitStand(r,plan_options)
      if nargin < 2
        plan_options = struct();
      end
      
      obj.plan_options = plan_options;
      if ~isfield(plan_options,'chair_height'), obj.plan_options.chair_height = 0.5; end
      if ~isfield(plan_options, 'use_mex'), obj.plan_options.use_mex = 1; end
      if ~isfield(plan_options,'speed'), obj.plan_options.speed = 1; end
      if ~isfield(plan_options,'back_gaze_bound'), obj.plan_options.back_gaze_bound = 0.4; end
      if ~isfield(plan_options,'min_distance'), obj.plan_options.min_distance = 0.1; end
      if ~isfield(plan_options,'foot_air'), obj.plan_options.foot_air = 'left'; end
      if ~isfield(plan_options,'foot_height'), obj.plan_options.foot_height = 0.2; end
      if ~isfield(plan_options,'back_bkz_weight'), obj.plan_options.back_bkz_weight = 1; end
      if ~isfield(plan_options,'pelvis_gaze_bound') obj.plan_options.pelvis_gaze_bound = 0.1; end
      if ~isfield(plan_options,'pelvis_gaze_bound') obj.plan_options.pelvis_gaze_angle = 0; end
      if ~isfield(plan_options,'sit_back_distance') obj.plan_options.sit_back_distance = 0.2; end
      if ~isfield(plan_options,'bky_angle'), obj.plan_options.bky_angle = -0.2; end
      if ~isfield(plan_options,'pelvis_contact_angle'), obj.plan_options.pelvis_contact_angle = 0; end
        
      if ~isfield(plan_options,'back_gaze_tight')
        obj.plan_options.back_gaze_tight.bound = 0.02;
        obj.plan_options.back_gaze_tight.angle = 0.2;
      end

      if isfield(plan_options,'pelvis_contact_angle') && obj.plan_options.pelvis_contact_angle
        obj.pelvis_contacts = {'l_fpelvis','r_fpelvis','m_pelvis'};
      else
        obj.pelvis_contacts = {'l_fpelvis','r_fpelvis'};
      end

      obj.r = r;
      obj.nq = r.getNumPositions();
      obj.handle = addpathTemporary([getenv('DRC_BASE'),'/software/control/matlab/planners/prone']);
      obj.kpt = KinematicPoseTrajectory(r,{});
      obj.kpt = obj.kpt.useHandGuards();
      obj.rpp = RobotPlanPublisher('CANDIDATE_ROBOT_PLAN_WITH_SUPPORTS',true,r.getStateFrame.coordinates(1:obj.nq));
      obj = obj.initialize();
    end

    function obj = initialize(obj)
      kpt = obj.kpt;
      r = obj.r;
      obj.nq = r.getNumPositions;
      obj.pose_data = load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/poses_v5.mat']);

      %% Torque Constraint
      joint_names = kpt.robot.getPositionFrame.coordinates;
      idx_arm = ~cellfun('isempty',strfind(joint_names,'arm'));
      idx_back = ~cellfun('isempty',strfind(joint_names,'back'));
      % idx = or(idx_arm,idx_back);
      idx = or(idx_arm,idx_back);
      names_arm_back = joint_names(idx);
      torque_multiplier = 0.7;
      torque_multiplier_back = 1;
      pmin = Point(obj.kpt.robot.getInputFrame,r.umin);
      pmax = Point(obj.kpt.robot.getInputFrame,r.umax);
      
      
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
        if strfind(name,'back_bky')
          lb(j) = -290;
          ub(j) = 290;
        end
        if strfind(name,'back_bkx')
          lb(j) = -190;
          ub(j) = 190;
        end
      end
      obj.torque_constraint = GravityCompensationTorqueConstraint(kpt.robot,joint_idx,lb,ub);

      %% Back gaze constraint
      obj.back_gaze_constraint = WorldGazeDirConstraint(kpt.robot,kpt.robot.findLinkId('utorso'),[0;0;1],[0;0;1],obj.plan_options.back_gaze_bound);
      %% Back gaze constraint
      theta = obj.plan_options.back_gaze_tight.angle;
      direction = [sin(theta);0;cos(theta)];
      obj.back_gaze_constraint_tight = WorldGazeDirConstraint(kpt.robot,kpt.robot.findLinkId('utorso'),[0;0;1],direction,obj.plan_options.back_gaze_tight.bound);

      %% Vertical back gaze constraint
      direction = [0;0;1];
      obj.back_vertical_gaze_constraint = WorldGazeDirConstraint(kpt.robot,kpt.robot.findLinkId('utorso'),[0;0;1],direction,0.01);

      %% Back gaze constraint
      theta = obj.plan_options.pelvis_gaze_angle;
      direction = [sin(theta);0;cos(theta)];
      obj.pelvis_gaze_constraint = WorldGazeDirConstraint(kpt.robot,kpt.robot.findLinkId('pelvis'),[0;0;1],direction,obj.plan_options.pelvis_gaze_bound);


      joint_min = [1;-Inf];
      joint_max = [Inf;-1];
      joint_ind = [kpt.robot.findPositionIndices('r_arm_shz');kpt.robot.findPositionIndices('l_arm_shz')];
      obj.shoulder_in_constraints = PostureConstraint(kpt.robot);
      obj.shoulder_in_constraints = obj.shoulder_in_constraints.setJointLimits(joint_ind,joint_min,joint_max);


      % joint_angles_r = [-2.306710720062256;
      %    0.14479857683181763;
      %    0.16161412000656128;
      %    0.767612934112548; 
      %    1.5669173002243042; 
      %    3];

      joint_angles_l = [2.2760796546936035, 0.3644244372844696,0.0,0.784614622592926,...
      -1.3069056272506714,-1.4836206436157227,0.08796451985836029]';
       
      % names_r = {'r_arm_elx','r_arm_ely','r_arm_mwx','r_arm_shx','r_arm_shz',...
      %   'r_arm_uwy'};

      names_l = {'l_arm_elx','l_arm_ely','l_arm_lwy','l_arm_mwx','l_arm_shx','l_arm_shz',...
        'l_arm_uwy'};

      joint_ind_r =zeros(length(names_l),1);
      joint_ind_l = zeros(length(names_l),1);
      joint_angles_r = joint_angles_l;
      for j = 1:numel(names_l)
        name_l = names_l{j};
        joint_ind_l(j) = r.findPositionIndices(name_l);
        name_r = name_l;
        name_r(1) = 'r';
        joint_ind_r(j) = r.findPositionIndices(name_r);

        % if joint is not named y then we need to flip the joint angle
        if ~strcmp(name_r(end),'y')
          joint_angles_r(j) = -joint_angles_r(j);
        end
      end

      joint_ind = [joint_ind_r;joint_ind_l];
      joint_angles = [joint_angles_r;joint_angles_l];
      obj.hand_up_tuck_constraint = PostureConstraint(kpt.robot);
      obj.hand_up_tuck_constraint = obj.hand_up_tuck_constraint.setJointLimits(joint_ind,joint_angles,joint_angles);

      joint_min = [-0.02];
      joint_max = [0.02];
      obj.back_z_constraint = PostureConstraint(kpt.robot);
      obj.back_z_constraint = obj.back_z_constraint.setJointLimits(kpt.robot.findPositionIndices('back_bkz'),joint_min,joint_max);

      joint_min = -0.1;
      joint_max = 0;
      obj.back_z_constraint_soft = PostureConstraint(kpt.robot);
      obj.back_z_constraint_soft = obj.back_z_constraint_soft.setJointLimits(kpt.robot.findPositionIndices('back_bkz'),joint_min,joint_max);

      joint_min = obj.plan_options.bky_angle;
      joint_max = obj.plan_options.bky_angle;
      obj.back_y_constraint = PostureConstraint(kpt.robot);
      obj.back_y_constraint = obj.back_y_constraint.setJointLimits(kpt.robot.findPositionIndices('back_bky'),joint_min,joint_max);

      %% Collision Constraint
      aco.body_idx = [2:kpt.robot.getNumBodies];
      obj.min_distance_constraint = MinDistanceConstraint(kpt.robot,obj.plan_options.min_distance,aco);

      %% Load the fixed point
      %% Nominal standing pose
      fixed_point_file = [getenv('DRC_BASE'),'/software/control/matlab/data/atlas_v5_fp.mat'];
      atlas_fp = load(fixed_point_file);
      obj.xstar = atlas_fp.xstar;
      obj.qstar = obj.xstar(1:obj.nq);
      obj.Q = eye(obj.nq);
      obj.Q(1:6) = zeros(6,1);
      obj.Q(1:6) = 0;
      obj.back_idx = obj.r.findPositionIndices('back');
      obj.arm_idx = obj.r.findPositionIndices('arm');

      % arm nominal posture constraint
      obj.arm_nominal_posture_constraint = PostureConstraint(kpt.robot);
      obj.arm_nominal_posture_constraint = obj.arm_nominal_posture_constraint.setJointLimits(obj.arm_idx,obj.pose_data.squat(obj.arm_idx),obj.pose_data.squat(obj.arm_idx));

      % sitting data
      data = load([getenv('DRC_BASE'),'/software/control/matlab/planners/chair_standup/chair_data_v5.mat']);
      obj.q_sol = data.q_sol;

      %% Max joint velocities
      max_arm_degrees_per_second = 35;
      max_degrees_per_second = 20;
      max_back_degrees_per_second = 8;
      max_base_meters_per_second = 0.05;
      back_xz_idx = [r.findPositionIndices('back_bkx'),r.findPositionIndices('back_bky')];
      joint_v_max = obj.plan_options.speed*repmat(max_degrees_per_second*pi/180, r.getNumVelocities()-3, 1);
      xyz_v_max = repmat(max_base_meters_per_second,3,1);
      obj.qd_max = [xyz_v_max;joint_v_max];
      obj.qd_max(obj.arm_idx) = obj.plan_options.speed*repmat(max_arm_degrees_per_second*pi/180, length(obj.arm_idx), 1);
      obj.qd_max(back_xz_idx) = obj.plan_options.speed*repmat(max_back_degrees_per_second*pi/180,1,2);

      %% Setting for pelvis bodies and pelvis_contact_pts, depends on which pelvis mode we are using . . . 
      if obj.plan_options.pelvis_contact_angle
        obj.pelvis_bodies = repmat(r.findLinkId('pelvis'),1,3);
        obj.pelvis_contact_pts = {obj.kpt.c('l_fpelvis'),obj.kpt.c('r_fpelvis'),obj.kpt.c('m_pelvis')};
      else
        obj.pelvis_bodies = repmat(r.findLinkId('pelvis'),1,3);
        middle_pelvis_contact_pt = 1/2.*(kpt.c('l_fpelvis') + kpt.c('r_fpelvis'));
        middle_pelvis_contact_pt(1) = middle_pelvis_contact_pt(1) - 0.02;
        obj.pelvis_contact_pts = {obj.kpt.c('l_fpelvis'),obj.kpt.c('r_fpelvis'),middle_pelvis_contact_pt};
      end

      obj.back_bkz_idx = obj.r.findPositionIndices('back_bkz');

      obj.elbow_ind = [r.findPositionIndices('r_arm_elx');r.findPositionIndices('r_arm_ely');r.findPositionIndices('l_arm_elx');r.findPositionIndices('l_arm_ely')];
      obj.elbow_angles = [-2.228;0.581;2.228;0.581];

    end

    function [qtraj,supports,support_times] = planSitting(obj,x0,plan_type)

      r = obj.r;
      nq = obj.nq;
      kpt = obj.kpt;
      failed_constraint_flag = 0;
      q0 = x0(1:obj.nq);
      q_sol = obj.q_sol;
      
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
      hand_above_ground_constraints = {l_hand_above_ground,r_hand_above_ground};

      %% Hands, pelvis position constraints relative to feet, will use the right foot for now
      kinsol = r.doKinematics(q0);
      T_foot = poseRPY2tform(r.forwardKin(kinsol,r.findLinkId('r_foot'),[0;0;0],1));
      ub = [-0.05; nan; nan];
      lb = [-Inf;nan;nan];
      pelvis_x_constraint = WorldPositionInFrameConstraint(kpt.robot,r.findLinkId('pelvis'),[0;0;0],T_foot,lb,ub);


      % constraint on pelvis position, only used in the sitdown portion of planning
      T_pelvis = poseRPY2tform(r.forwardKin(kinsol,r.findLinkId('pelvis'),[0;0;0],1));
      lb = [-(obj.plan_options.sit_back_distance + 0.2);-0.02;nan];
      ub = [-(obj.plan_options.sit_back_distance);0.02;nan];
      pelvis_xy_constraint = WorldPositionInFrameConstraint(kpt.robot,r.findLinkId('pelvis'),[0;0;0],T_pelvis,lb,ub);

      % don't yaw from current orientation
      yaw_constraint = PostureConstraint(kpt.robot);
      yaw_constraint = yaw_constraint.setJointLimits(6,q0(6),q0(6));

      lb = [0.2;nan;nan];
      ub = [Inf;nan;nan];
      l_hand_x_constraint = WorldPositionInFrameConstraint(kpt.robot,r.findLinkId('l_hand'),[0;0;0],T_foot,lb,ub);
      r_hand_x_constraint = WorldPositionInFrameConstraint(kpt.robot,r.findLinkId('r_hand'),[0;0;0],T_foot,lb,ub);
      x_position_constraints = {pelvis_xy_constraint,l_hand_x_constraint,r_hand_x_constraint};
      x_hand_position_constraints = {l_hand_x_constraint,r_hand_x_constraint};

      
      kinsol = r.doKinematics(q0);
      l_foot_pos = r.forwardKin(kinsol,kpt.linkId('l_foot'),kpt.c('l_foot'));
      r_foot_pos = r.forwardKin(kinsol,kpt.linkId('r_foot'),kpt.c('r_foot'));
      ground_height = 1/2*(mean(l_foot_pos(3,:)) + mean(r_foot_pos(3,:)));
      pelvis_height = obj.plan_options.chair_height + ground_height;
      
      %% Pelvis height constraints
      lb = [nan;nan;pelvis_height];
      ub = [nan;nan;pelvis_height];
      lb = repmat(lb,1,size(kpt.c('l_fpelvis'),2));
      ub = repmat(ub,1,size(kpt.c('l_fpelvis'),2));
      
      l_pelvis_height = WorldPositionConstraint(kpt.robot,kpt.linkId('l_fpelvis'),kpt.c('l_fpelvis'),lb,ub);
      r_pelvis_height = WorldPositionConstraint(kpt.robot,kpt.linkId('r_fpelvis'),kpt.c('r_fpelvis'),lb,ub);
      
      %% Weighting matrix
      Q = obj.Q;
      
      if strcmp(plan_type,'sit') || strcmp(plan_type, 'squat')
        
        
        
        %% Sitting with COM over feet
        clear options;
        options = obj.plan_options;
        options.constraints = [{obj.torque_constraint,obj.back_gaze_constraint,obj.back_z_constraint_soft,...
        obj.pelvis_gaze_constraint,yaw_constraint,obj.min_distance_constraint},x_position_constraints,hand_above_ground_constraints];
        options.no_movement.bodies = {'l_foot','r_foot'};
        options.no_movement.q = q0;
        options.qs_contacts = {'l_foot','r_foot'};
        options.Q = Q;

        options.height.names = obj.pelvis_contacts;
        options.height.heights = repmat({pelvis_height},size(obj.pelvis_contacts));

        q_nom = obj.pose_data.squat;
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        q_nom(obj.back_idx) = obj.qstar(obj.back_idx);
        q_nom(obj.arm_idx) = obj.qstar(obj.arm_idx);

        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_sitting_feet = q;

        if info < 10
          disp('solved squat with no errors');
        end

        %% Intermediate posture to avoid arm collisions
        clear options;
        options = obj.plan_options;
        options.constraints = {obj.min_distance_constraint,obj.torque_constraint,obj.back_gaze_constraint_tight, obj.back_z_constraint, ...
        obj.arm_nominal_posture_constraint};
        options.no_movement.bodies = {'l_foot','r_foot','pelvis'};
        options.no_movement.q = q_sitting_feet;

        % increase the shrink factor 
        options.shrink_factor = 0.9;
        options.Q = Q;
        options.qs_contacts = {'l_foot','r_foot','l_fpelvis','r_fpelvis','m_pelvis'};
        % rotate to align it with the current position
        q_nom = obj.pose_data.squat;
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        q_nom(obj.back_idx) = obj.qstar(obj.back_idx);
        q_nom(obj.arm_idx) = obj.qstar(obj.arm_idx);
        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_intermediate = q;

        %% Find the appropriate intermediate postures
        q_arm_prep = q_intermediate;
        q_arm_prep(obj.elbow_ind) = obj.elbow_angles;
        q_hand_up_tuck = q_intermediate;
        q_hand_up_tuck(obj.arm_idx) = obj.pose_data.arm_up_tuck(obj.arm_idx);

        % sitting is the same as arm up tuck but with the back tilted back
        q_sitting = q_hand_up_tuck;
        q_sitting(obj.back_idx) = zeros(size(obj.back_idx));
        q_sitting(obj.r.findPositionIndices('back_bky')) = obj.plan_options.bky_angle;

        
        q_vals = [q0,q_sitting_feet,q_intermediate,q_arm_prep,q_hand_up_tuck,q_sitting];
        [qtraj,support_times] = obj.constructAndSmoothTrajectory(q_vals);
        support_times = [support_times(1:2),support_times(end)];
        qtraj = obj.touchUpTrajectory(qtraj);
        
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
        supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
        
        supports(2).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),obj.pelvis_bodies];
        supports(2).contact_pts = [{kpt.c('l_foot'),kpt.c('r_foot')},obj.pelvis_contact_pts];
        
        supports(3) = supports(2);
        
        %% Return a squating plan rather than a full sitdown plan
        if strcmp(plan_type,'squat')
          q_vals = [q0,q_sitting_feet];
          [qtraj,support_times] = obj.constructAndSmoothTrajectory(q_vals);
          qtraj = obj.touchUpTrajectory(qtraj);
          supports = struct('bodies',{},'contact_pts',{});
          supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
          supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
          supports(2) = supports(1);
        end
        
      end
      
      if strcmp(plan_type,'stand') || strcmp(plan_type,'stand_from_squat')

        %% Intermediate posture to avoid arm collisions
        clear options;
        options = obj.plan_options;
        options.constraints = [{obj.min_distance_constraint,obj.torque_constraint,obj.back_gaze_constraint_tight,obj.arm_nominal_posture_constraint}];
        options.no_movement.bodies = {'l_foot','r_foot','pelvis'};
        options.no_movement.q = q0;

        % increase the shrink factor 
        options.shrink_factor = 0.9;
        options.Q = Q;
        options.qs_contacts = {'l_foot','r_foot','l_fpelvis','r_fpelvis','m_pelvis'};
        % rotate to align it with the current position
        q_nom = q0;
        q_nom(obj.back_idx) = obj.qstar(obj.back_idx);
        q_nom(obj.arm_idx) = obj.qstar(obj.arm_idx);
        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_intermediate = q;

        %% Sitting with COM over feet
        disp('solving for sitting with COM over feet');
        clear options;
        options = obj.plan_options;
        options.constraints = [{obj.torque_constraint,obj.back_gaze_constraint,obj.min_distance_constraint,obj.back_z_constraint_soft},...
        x_hand_position_constraints,hand_above_ground_constraints];
        options.no_movement.bodies = {'pelvis','l_foot','r_foot'};
        options.no_movement.q = q0;
        
        q_nom = obj.pose_data.squat;
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        q_nom(obj.back_idx) = obj.qstar(obj.back_idx);
        q_nom(obj.arm_idx) = obj.qstar(obj.arm_idx);
        options.Q = Q;
        
        options.qs_contacts = {'l_foot','r_foot'};
        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_sitting_feet = q;
        
        %% Standing in nominal pose
        clear options;
        disp('solving for standing pose')
        contacts = {'l_foot','r_foot'};
        options.constraints = {obj.back_vertical_gaze_constraint,obj.min_distance_constraint};
        options.no_movement.bodies = {'l_foot','r_foot'};
        options.no_movement.q = q0;
        options.qs_contacts = {'l_foot','r_foot'};
        options.Q = Q;
        q_nom = obj.qstar;
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_standing = q;

        

        %% A few intermediate poses
        q_arm_prep = q_intermediate;
        q_arm_prep(obj.elbow_ind) = obj.elbow_angles;
        q_hand_up_tuck = q_intermediate;
        q_hand_up_tuck(obj.arm_idx) = obj.pose_data.arm_up_tuck(obj.arm_idx);

        q_vals = [q0, q_hand_up_tuck, q_arm_prep, q_intermediate, q_sitting_feet, q_standing];
        [qtraj,support_times] = obj.constructAndSmoothTrajectory(q_vals);
        support_times = [support_times(1),support_times(4:5)];
        qtraj = obj.touchUpTrajectory(qtraj);
             
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),obj.pelvis_bodies];
        supports(1).contact_pts = [{kpt.c('l_foot'),kpt.c('r_foot')},obj.pelvis_contact_pts];
        
        supports(2).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
        supports(2).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
        
        supports(3) = supports(2);
        
        % just use q_2, which is the standing pose that we just found
        if strcmp(plan_type,'stand_from_squat')
          q_vals = [q0,q_standing];
          [qtraj,support_times] = obj.constructAndSmoothTrajectory(q_vals);
          qtraj = obj.touchUpTrajectory(qtraj);
          supports = struct('bodies',{},'contact_pts',{});
          supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
          supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
          supports(2) = supports(1);
        end
      end

      if strcmp(plan_type,'hold_with_pelvis_contact')
        support_times = [0,10];
        qtraj = PPTrajectory(foh(support_times,[q0,q0]));
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),obj.pelvis_bodies];
        supports(1).contact_pts = [{kpt.c('l_foot'),kpt.c('r_foot')},obj.pelvis_contact_pts];
        supports(2) = supports(1);
      end

      if strcmp(plan_type,'hold_without_pelvis_contact')
        support_times = [0,10];
        qtraj = PPTrajectory(foh(support_times,[q0,q0]));
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot')];
        supports(1).contact_pts = {kpt.c('l_foot'),kpt.c('r_foot')};
        supports(2) = supports(1);
      end


      if strcmp(plan_type,'sit_from_current')
        clear options;
        options = obj.plan_options;
        options.constraints = [{obj.torque_constraint,obj.back_gaze_constraint_tight,obj.min_distance_constraint},x_position_constraints,hand_above_ground_constraints];
        options.no_movement.bodies = {'l_foot','r_foot','pelvis'};
        options.no_movement.q = q0;
        options.Q = Q;
        options.qs_contacts = {'l_foot','r_foot','l_fpelvis','r_fpelvis'};
        q_nom = q_sol(:,1);
        
        % rotate to align it with the current position
        q_nom(1:2) = q0(1:2);
        q_nom(6) = q0(6);
        q_nom(obj.back_idx) = obj.qstar(obj.back_idx);
        [q,info,infeasible_constraint] = kpt.inverseKin(q_nom,options);
        info
        infeasible_constraint
        if ~isempty(infeasible_constraint), failed_constraint_flag = 1; end
        q_sitting = q;

        q_vals = [q0,q_sitting];
        [qtraj,support_times] = obj.constructAndSmoothTrajectory(q_vals);
        qtraj = obj.touchUpTrajectory(qtraj);
             
        supports = struct('bodies',{},'contact_pts',{});
        supports(1).bodies = [r.findLinkId('l_foot'),r.findLinkId('r_foot'),obj.pelvis_bodies];
        supports(1).contact_pts = [{kpt.c('l_foot'),kpt.c('r_foot')},obj.pelvis_contact_pts];
        supports(2) = supports(1);
      end

      % print whether all constraints satisfied
      if failed_constraint_flag
        disp('NOT ALL CONSTRAINTS SATISFIED');
      else
        disp('all constraints satisfied');
      end

      % publish the plan
      obj.publish(qtraj,supports,support_times,failed_constraint_flag)
    end

    function publish(obj,qtraj,supports,support_times,failed_constraint_flag)
      if failed_constraint_flag
        snopt_info = 20;
      else
        snopt_info = 1;
      end

      T = qtraj.getBreaks();
      Q = qtraj.eval(T);
      X = [Q;0*Q]; % give it zero velocity
      obj.rpp.publishPlanWithSupports(X,T,supports,support_times, snopt_info);
    end

    function [qtraj,support_times] = constructAndSmoothTrajectory(obj,q_vals)
      kpt = obj.kpt;
      r = obj.r;
      speed = obj.plan_options.speed;
      N = size(q_vals,2);
      qtraj_cell = {};

      for j = 1:N-1
        qtraj_cell{j} = kpt.constructTrajectory(q_vals(:,j:j+1));
        qtraj_cell{j} = rescalePlanTiming(qtraj_cell{j},obj.qd_max);
      end

      qtraj = qtraj_cell{1};
      support_times = qtraj.tspan;

      for j = 2:numel(qtraj_cell)
        qtraj_cell{j} = qtraj_cell{j}.shiftTime(qtraj.tspan(2));
        qtraj = qtraj.append(qtraj_cell{j});
        support_times(end+1) = qtraj.tspan(2);
      end
    end

    function qtraj_new = touchUpTrajectory(obj,qtraj,tspan)
      r = obj.r;

      if nargin < 3
        tspan = struct();
      end

      if ~isfield(tspan,'r_foot'), tspan.r_foot = qtraj.tspan; end
      if ~isfield(tspan,'l_foot'), tspan.l_foot = qtraj.tspan; end

      t0 = qtraj.tspan(1);
      tf = qtraj.tspan(2);
      
      q0 = qtraj.eval(t0);
      ts = linspace(t0,tf,20);
      q_seed = qtraj.eval(ts);
      pts = [1,-1,0; 0, 0, 1 ; 0,0,0];
      
      kinsol_r = r.doKinematics(qtraj.eval(tspan.r_foot(1)));
      r_foot_position = r.forwardKin(kinsol_r,r.findLinkId('r_foot'),pts);
      kinsol_l = r.doKinematics(qtraj.eval(tspan.l_foot(1)));
      l_foot_position = r.forwardKin(kinsol_l,r.findLinkId('l_foot'),pts);
      
      r_foot_constraint = WorldPositionConstraint(r,r.findLinkId('r_foot'),pts,r_foot_position,r_foot_position,tspan.r_foot);
      l_foot_constraint = WorldPositionConstraint(r,r.findLinkId('l_foot'),pts,l_foot_position,l_foot_position,tspan.l_foot);
      constraints = {l_foot_constraint,r_foot_constraint};
      
      [q_sol,info,infeasible_constraint] = r.inverseKinPointwise(ts,q_seed,q_seed,constraints{:});
      disp('touching up the trajectory')
      

      if any(info > 10)
        disp('initial solve yielded')
        info
        infeasible_constraint
        disp('some knot points dont satisfy all constraints, removing them');
        I = find(info < 10);
        q_sol = q_sol(:,I);
        ts = ts(I);
      end
      qtraj_new = PPTrajectory(pchip(ts,q_sol));
    end

  end

  
  methods (Static)

    function [qtraj,supports,support_times] = plan(r,x0,plan_type,plan_options)
      obj = PlanSitStand(r,plan_options);

      if any(strcmp(plan_type,{'sit','stand','squat','stand_from_squat','sit_from_current',...
        'hold_with_pelvis_contact','hold_without_pelvis_contact'}))
        [qtraj,supports,support_times] = obj.planSitting(x0,plan_type);
      elseif any(strcmp(plan_type,{'one_leg_stand','stand_from_one_leg','lean'}))
        [qtraj,supports,support_times] = obj.planOneLeg(x0,plan_type);
      end
    end
    

    function constraint = foot_air_to_ground_constraint(r,kpt,q0,foot_ground,foot_air)
      foot_width = 0.22;
      tol = 0.02;

      pts = [1,-1,0;0,0,1;0,0,0];
      kinsol = r.doKinematics(q0);
      foot_pos_ground = r.forwardKin(kinsol,r.findLinkId(foot_ground),pts);

      bds = foot_pos_ground(1:3,:);
      if strcmp(foot_ground,'r_foot')
        bds(2,:) = bds(2,:) + foot_width;
      else
        bds(2,:) = bds(2,:) - foot_width;
      end

      tol_bds = tol*ones(size(pts));
      tol_bds(3,:) = 0*tol_bds(:,3);
      lb = bds - tol_bds;
      ub = bds + tol_bds;
      
      T_foot = poseRPY2tform(foot_pos_ground);
      constraint = WorldPositionInFrameConstraint(kpt.robot,r.findLinkId(foot_air),pts,T_foot,lb,ub);
    end

    function constraint = foot_on_ground_constraint(r,kpt,q0,foot_ground,foot_air)
      foot_width = 0.22;
      tol = 0.02;

      pts = [1,-1,0;0,0,1;0,0,0];
      kinsol = r.doKinematics(q0);
      foot_pos_ground = r.forwardKin(kinsol,r.findLinkId(foot_ground),pts);

      bds = foot_pos_ground(1:3,:);
      bds(1:2,:) = nan*bds(1:2,:);
      
      T_foot = poseRPY2tform(foot_pos_ground);
      constraint = WorldPositionConstraint(kpt.robot,r.findLinkId(foot_air),pts,bds,bds);
    end    
    
  end
end













