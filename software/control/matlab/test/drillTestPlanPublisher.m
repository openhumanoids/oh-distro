classdef drillTestPlanPublisher
  %NOTEST
  % A testing class for generating and publishing (by LCM) plans
  % to attempt the drill
  % General sequence:
  %   -Costruct a drillTestPlanPublisher
  %   -createInitialReachPlan (reach to pre-drill pose)
  %   -createDrillingPlan (drill in)
  %   -createCircleCutPlan (cut a circle)
  % todo: zero velocity constraints?
  % todo: look into the fixed initial state option
  properties
    r
    atlas
    plan_pub
    pose_pub
    v
    drill_pt_on_hand
    drill_axis_on_hand
    drill_dir_des
    drill_dir_threshold
    hand_body = 29;
    joint_indices;
    ik_options
    free_ik_options
    drilling_world_axis
    doVisualization = true;
    doPublish = false;
    default_axis_threshold = 2*pi/180;
    atlas2robotFrameIndMap
    footstep_msg
    lc
    lcmgl
  end
  
  methods
    function obj = drillTestPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand, ...
        drilling_world_axis, drill_dir_des, drill_dir_threshold, doVisualization, doPublish)
      obj.atlas = atlas;
      obj.r = r;
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
      end
      if nargin < 6
        obj.doVisualization = true; % default
      else
        obj.doVisualization = doVisualization;
      end
      if nargin < 7
        obj.doPublish = false; % default
      else
        obj.doPublish = doPublish;
      end
      joint_names = obj.atlas.getStateFrame.coordinates(1:getNumDOF(obj.atlas));
      jposoint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
      cost = ones(34,1);
      cost(1:6) = 10*ones(6,1);
      cost(7:9) = [100;1000;100];
      
      vel_cost = cost*.05;
      accel_cost = cost*.05;
      
      obj.footstep_msg = drc.walking_goal_t();
      obj.footstep_msg.max_num_steps = NaN;
      obj.footstep_msg.min_num_steps = NaN;
      obj.footstep_msg.timeout = NaN;
      obj.footstep_msg.step_speed = NaN;
      obj.footstep_msg.nom_step_width = NaN;
      obj.footstep_msg.nom_forward_step = NaN;
      obj.footstep_msg.max_forward_step = NaN;
      obj.footstep_msg.step_height = NaN;
      obj.footstep_msg.fixed_step_duration = NaN;
      obj.footstep_msg.bdi_step_duration = NaN;
      obj.footstep_msg.bdi_sway_duration = NaN;
      obj.footstep_msg.bdi_lift_height = NaN;
      obj.footstep_msg.bdi_toe_off = NaN;
      obj.footstep_msg.bdi_knee_nominal = NaN;
      obj.footstep_msg.follow_spline = false;
      obj.footstep_msg.ignore_terrain = false;
      obj.footstep_msg.behavior = NaN;
      obj.footstep_msg.mu = NaN;
      obj.footstep_msg.allow_optimization = true;
      obj.footstep_msg.is_new_goal = true;
      obj.footstep_msg.right_foot_lead = true;
      obj.footstep_msg.map_command = NaN;
      obj.footstep_msg.goal_pos = drc.position_3d_t();
      obj.footstep_msg.goal_pos.translation = drc.vector_3d_t();
      obj.footstep_msg.goal_pos.rotation = drc.quaternion_t();

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmgl = drake.util.BotLCMGLClient(obj.lc,'drill_planned_path');

      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setDebug(true);
      iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
      iktraj_options = iktraj_options.setQa(diag(vel_cost));
      iktraj_options = iktraj_options.setQv(diag(accel_cost));
      iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1)); % upper and lower bnd on velocity.
      iktraj_options = iktraj_options.setMajorIterationsLimit(3000);
      iktraj_options = iktraj_options.setMex(true);
      iktraj_options = iktraj_options.setMajorOptimalityTolerance(1e-5);
      
      obj.ik_options = iktraj_options;
      obj.free_ik_options = iktraj_options.setFixInitialState(false);
      
      obj.drill_pt_on_hand = drill_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
      obj.drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
      obj.drill_dir_des = drill_dir_des/norm(drill_dir_des);
      obj.drill_dir_threshold = drill_dir_threshold;
      
      obj.joint_indices = [7:9 22:26 33];
      
      valuecheck(obj.drill_dir_des'*obj.drill_axis_on_hand,0);
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end
    
    
    % search for a posture for the robot to go through the segments
    function [xtraj,snopt_info,infeasible_constraint] = findDrillingMotion(obj, q0, drill_points, free_body_pose)
      n_points = size(drill_points,2);
      sizecheck(drill_points, [3 n_points]);
      
      T = 1;
      N = 3*n_points;
      
      t_vec = linspace(0,T,N);
      t_drill = linspace(0,T,n_points);
      
      % gaze constraint for drill
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);

      posture_constraint = PostureConstraint(obj.r);
      if free_body_pose
        % create posture constraint.  allow moving x, y, z, yaw
        posture_index = setdiff((1:obj.r.num_q)',[1 2 3 6 obj.joint_indices]');
        posture_constraint = posture_constraint.setJointLimits(3, .5, .8);
        posture_constraint = posture_constraint.setJointLimits(6, q0(6) - .2, q0(6) + .2);
      else
        posture_index = setdiff((1:obj.r.num_q)',[obj.joint_indices]');
      end
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits([7:9]', [-.2 -.2 -.2]', [.2 .2 .2]');
            
      % create drill position constraints
      x_drill_traj = PPTrajectory(foh(t_drill,drill_points));
      drill_pos_constraint = cell(1,N);
      for i=1:N,
        drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,...
          x_drill_traj.eval(t_vec(i)),x_drill_traj.eval(t_vec(i)),[t_vec(i) t_vec(i)]);
      end
      
      % fix body pose for all t
      body_pose_constraint = WorldFixedBodyPoseConstraint(obj.r,2); % fix the pelvis
      
      q_init = q0 + .1*randn(obj.r.num_q,1);
      q_nom = zeros(obj.r.num_q,N);
      for i=1:N,
        [q_nom(:,i), snopt_info_ik(i),infeasible_constraint_ik] = inverseKin(obj.r,q_init,q_init,...
          drill_pos_constraint{i}, drill_dir_constraint, posture_constraint, obj.ik_options);
        
        if(snopt_info_ik(i) > 10)
          send_msg = sprintf('snopt_info = %d. The IK fails for t=%f\n', snopt_info_ik(i), t_vec(i));
          send_status(4,0,0,send_msg);
          display(infeasibleConstraintMsg(infeasible_constraint_ik));
          warning(send_msg);
        end
        q_nom(:,i) = q_nom(:,i) +  + .1*randn(obj.r.num_q,1);
        q_init = q_nom(:,i);
      end
      qtraj_guess = PPTrajectory(foh(t_vec,q_nom));
      
      obj.free_ik_options.setMajorOptimalityTolerance(1e-4);
      obj.free_ik_options = obj.free_ik_options.setMajorIterationsLimit(2000);
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,body_pose_constraint,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.free_ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && (snopt_info <= 10 || snopt_info == 32)
        obj.v.playback(xtraj);
      end
      
      % really need to just publish this as ghosts
      if obj.doPublish && (snopt_info <= 10 || snopt_info == 32)
%         obj.publishPoseTraj(xtraj);
        obj.publishTraj(xtraj,snopt_info);
        if free_body_pose
          % also publish a walking plan
          x_end = xtraj.eval(T);
          pose = [x_end(1:3); rpy2quat(x_end(4:6))];
          obj.publishWalkingGoal(pose);
        end
      
      
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createInitialReachPlan(obj, q0, x_drill, first_cut_dir, T)
      N = 5;
      t_vec = linspace(0,T,N);
      
      if ~isempty(first_cut_dir)
        % generate desired quaternion
        R_hand = [obj.drill_axis_on_hand obj.drill_dir_des cross(obj.drill_axis_on_hand, obj.drill_dir_des)];
        R_world = [obj.drilling_world_axis first_cut_dir cross(obj.drilling_world_axis, first_cut_dir)];
        %         R_rel = R_hand*R_world;
        R_rel = R_world*R_hand';
        
        quat_des = rotmat2quat(R_rel);
        
        % create drill direction constraint
        drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
          quat_des, obj.default_axis_threshold,obj.drill_dir_threshold,[t_vec(end) t_vec(end)]);
      else
        % create drill direction constraint
        drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
          obj.drilling_world_axis,obj.default_axis_threshold,[t_vec(end) t_vec(end)]);
      end
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill position constraint
      drill_pos_constraint = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill,x_drill,[t_vec(end) t_vec(end)]);
      
      % Find nominal pose
      diff_opt = inf;
      q_end_nom = q0;
      for i=1:50,
        [q_tmp,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0 + .1*randn(obj.r.num_q,1),q0,...
          drill_pos_constraint,drill_dir_constraint,posture_constraint,obj.ik_options);
        
        c_tmp = (q_tmp - q0)'*obj.ik_options.Q*(q_tmp - q0);
        if snopt_info_ik == 1 && c_tmp < diff_opt
          q_end_nom = q_tmp;
          diff_opt = c_tmp;
        end
      end
      
      if(diff_opt == inf)
        send_msg = sprintf('snopt_info = %d. The IK fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));
      
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint,drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createDrillingPlan(obj, q0, x_drill_final, first_cut_dir, T)
      N = 10;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
      
      t_vec = linspace(0,T,N);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      if ~isempty(first_cut_dir)
        % generate desired quaternion
        R_hand = [obj.drill_axis_on_hand obj.drill_dir_des cross(obj.drill_axis_on_hand, obj.drill_dir_des)];
        R_world = [obj.drilling_world_axis first_cut_dir cross(obj.drilling_world_axis, first_cut_dir)];
        R_rel = R_world*R_hand';
        quat_des = rotmat2quat(R_rel);
        
        % create drill direction constraint
        drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
          quat_des, obj.default_axis_threshold,obj.drill_dir_threshold);
      else
        % create drill direction constraint
        drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
          obj.drilling_world_axis,obj.default_axis_threshold);
      end
      
      % create drill position constraints
      x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      drill_pos_constraint = cell(1,N-1);
      for i=2:N,
        drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % Find nominal pose
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));
      
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createCircleCutPlan(obj, q0, circle_center, T)
      N = 10;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
      
      t_vec = linspace(0,T,N);
      qtraj_guess = PPTrajectory(foh([0 Ts],[q0, q0+.1*randn(length(q0),1)]));
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);
      
      % create drill position constraints
      rot_axis = obj.drilling_world_axis;
      radius_vec = x_drill_init - circle_center;
      radius_vec = radius_vec - rot_axis*(rot_axis'*radius_vec);
      radius = norm(radius_vec);
      theta = linspace(0,2*pi,N);
      
      rot_axis*rot_axis'*radius_vec*(1-cos(theta)) + radius_vec*cos(theta) + cross(rot_axis,radius_vec)*sin(theta);
      
      x_drill = repmat(x_drill_init - radius_vec,1,N) + ...
        rot_axis*rot_axis'*radius_vec*(1-cos(theta)) + radius_vec*cos(theta) + cross(rot_axis,radius_vec)*sin(theta);
      drill_pos_constraint = cell(1,N-1);
      for i=2:N,
        drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createLinePlan(obj, q0, x_drill_init, x_drill_final, T)
      N = 10;
      t_vec = linspace(0,T,N);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);
      
      % create drill position constraints
      x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      drill_pos_constraint = cell(1,N);
      for i=1:N,
        drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % Find nominal poses
      [q_start_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{1},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (init) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q_start_nom,q_start_nom,...
        drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      
      
      qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.free_ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint] = createDirectedLinePlan(obj, q0, x_drill_final, T)
      N = 10;
      t_vec = linspace(0,T,N);
      
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
      
      % find direction of drill motion
      drill_motion_dir = (x_drill_final - x_drill_init);
      drill_motion_dir = drill_motion_dir/norm(drill_motion_dir);
      valuecheck(drill_motion_dir'*obj.drilling_world_axis,0,1e-3);
      
      
      % generate desired quaternion
      R_hand = [obj.drill_axis_on_hand obj.drill_dir_des cross(obj.drill_axis_on_hand, obj.drill_dir_des)];
      R_world = [obj.drilling_world_axis drill_motion_dir cross(obj.drilling_world_axis, drill_motion_dir)];
      R_rel = R_world*R_hand';
      quat_des = rotmat2quat(R_rel);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',[obj.joint_indices]');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        quat_des, obj.default_axis_threshold,obj.drill_dir_threshold);
      
      % create drill position constraints
      x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      drill_pos_constraint = cell(1,N-1);
      for i=2:N,
        drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % Find nominal poses
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createConstrainedLinePlan(obj, q0, x_drill_init, x_drill_final, quat_des, threshold, T)
      N = 10;
      t_vec = linspace(0,T,N);
      
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',[1 2 3 6 obj.joint_indices]');
      posture_constraint_free_base = PostureConstraint(obj.r);
      posture_constraint_free_base = posture_constraint_free_base.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      %       WorldGazeOrientConstraint(obj.r,obj.hand_body,axis,quat_des,conethreshold,threshold,tspan)
      drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        quat_des, obj.default_axis_threshold,threshold);
      %         obj.drilling_world_axis,obj.default_axis_threshold);
      
      body_pose_constraint = WorldFixedBodyPoseConstraint(obj.r,2); % fix the pelvis
      
      % create drill position constraints
      x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      drill_pos_constraint = cell(1,N);
      for i=1:N,
        drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % Find nominal poses
      [q_start_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{1},drill_dir_constraint,posture_constraint_free_base,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (start) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q_start_nom(posture_index),q_start_nom(posture_index));
      
      %       posture_constraint = posture_constraint_free_base;
      
      % Find nominal poses
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      
      
      qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,body_pose_constraint,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint_free_base,obj.free_ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj, snopt_info, infeasible_constraint] = createGotoPlan(obj,q0,qf,T)
      N = 5;
      t_vec = linspace(0,T,N);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      final_posture_constraint = PostureConstraint(obj.r, [T T]);
      final_posture_constraint = final_posture_constraint.setJointLimits(obj.joint_indices', qf(obj.joint_indices), qf(obj.joint_indices));
      
      qtraj_guess = PPTrajectory(foh([0 T],[q0, qf]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        posture_constraint,final_posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function publishTraj(obj,xtraj,snopt_info)
      utime = now() * 24 * 60 * 60;
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
      xtraj_atlas(2+(1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
      snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));
      
      obj.plan_pub.publish(xtraj_atlas,ts,utime,snopt_info_vector);
      
      ts_line = linspace(xtraj.tspan(1),xtraj.tspan(2),200);
      x_line = xtraj.eval(ts_line);
      obj.lcmgl.glColor3f(1,0,0); 
      obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
      for i=1:length(ts_line),
        q_line = x_line(1:nq_atlas,i);
        kinsol = obj.r.doKinematics(q_line);
%         drill_pts = obj.r.forwardKin(kinsol,obj.hand_body,...
%           [obj.drill_pt_on_hand, obj.drill_pt_on_hand + .0254*obj.drill_axis_on_hand]);
%         obj.lcmgl.line3(drill_pts(1,1),drill_pts(2,1),drill_pts(3,1),...
%           drill_pts(1,2),drill_pts(2,2),drill_pts(3,2));
        
        drill_pt = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
        obj.lcmgl.glVertex3d(drill_pt(1),drill_pt(2),drill_pt(3));
      end
      obj.lcmgl.glEnd();
      obj.lcmgl.switchBuffers();
    end
    
    function publishPoseTraj(obj,xtraj)
      utime = now() * 24 * 60 * 60;
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2*nq_atlas,length(ts));
      xtraj_atlas((1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
      for i=1:length(ts),
        obj.pose_pub.publish(xtraj_atlas(:,i),utime);
      end
    end
    
    function publishWalkingGoal(obj,pose)
      obj.footstep_msg.utime = now() * 24 * 60 * 60;
      obj.footstep_msg.goal_pos.translation.x = pose(1);
      obj.footstep_msg.goal_pos.translation.y = pose(2);
      obj.footstep_msg.goal_pos.translation.z = pose(3);
      obj.footstep_msg.goal_pos.rotation.w = pose(4);
      obj.footstep_msg.goal_pos.rotation.x = pose(5);
      obj.footstep_msg.goal_pos.rotation.y = pose(6);
      obj.footstep_msg.goal_pos.rotation.z = pose(7);
      
      obj.lc.publish('WALKING_GOAL', obj.footstep_msg);      
    end
    
  end
end