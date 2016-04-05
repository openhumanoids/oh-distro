classdef drillPlanner
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
    posture_pub
    v
    drill_pt_on_hand
    drill_axis_on_hand
    hand_body;
    joint_indices;
    ik_options
    free_ik_options
    drilling_world_axis
    doVisualization = true;
    doPublish = false;
    default_axis_threshold = 5*pi/180;
    atlas2robotFrameIndMap
    footstep_msg
    lc
    state_monitor
    lcmgl
    allowPelvisHeight
  end

  methods
    function obj = drillPlanner(r,atlas,drill_pt_on_hand, drill_axis_on_hand, ...
        drilling_world_axis, useRightHand, doVisualization, doPublish, allowPelvisHeight)
      obj.atlas = atlas;
      obj.r = r;
      obj.doVisualization = doVisualization;
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
        obj.v.playback_speed = 5;
      end


      l_hand_frame = handFrame(2,'left');
      r_hand_frame = handFrame(2,'right');
      obj.posture_pub = PosturePlanner(r,atlas,l_hand_frame,r_hand_frame,2);


      obj.doPublish = doPublish;

      obj.allowPelvisHeight = allowPelvisHeight;

      joint_names = obj.atlas.getStateFrame.coordinates(1:getNumPositions(obj.atlas));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

      obj.doPublish = doPublish;
      obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);

      back_joint_indices = regexpIndex('^back_bk[x-z]$',r.getStateFrame.coordinates);

      if useRightHand
        obj.hand_body = regexpIndex('r_hand',{r.getBody(:).linkname});
        arm_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      else
        obj.hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
        arm_joint_indices = regexpIndex('^l_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      end


      cost = ones(34,1);
      cost([1 2 6]) = 50*ones(3,1);
      cost(3) = 10;
      cost(back_joint_indices) = [1000;2000;500];

      vel_cost = cost*.05;
      accel_cost = cost*.05;

      obj.footstep_msg = drc.walking_goal_t();
      obj.footstep_msg.max_num_steps = 10;
      obj.footstep_msg.min_num_steps = 0;
      obj.footstep_msg.timeout = 0;
      obj.footstep_msg.step_speed = 1;
      obj.footstep_msg.nom_step_width = .26;
      obj.footstep_msg.nom_forward_step = .15;
      obj.footstep_msg.max_forward_step = .45;
      obj.footstep_msg.step_height = .05;
      obj.footstep_msg.fixed_step_duration = 0;
      obj.footstep_msg.bdi_step_duration = 2;
      obj.footstep_msg.bdi_sway_duration = 0;
      obj.footstep_msg.bdi_lift_height = .05;
      obj.footstep_msg.bdi_toe_off = 1;
      obj.footstep_msg.bdi_knee_nominal = 0;
      obj.footstep_msg.bdi_max_foot_vel = 0;
      obj.footstep_msg.bdi_sway_end_dist = 0.02;
      obj.footstep_msg.bdi_step_end_dist = 0.02;
      obj.footstep_msg.follow_spline = false;
      obj.footstep_msg.behavior = drc.walking_goal_t.BEHAVIOR_BDI_STEPPING;
      obj.footstep_msg.mu = 1;
      obj.footstep_msg.allow_optimization = true;
      obj.footstep_msg.is_new_goal = true;
      obj.footstep_msg.right_foot_lead = drc.walking_goal_t.LEAD_AUTO;
      obj.footstep_msg.map_mode = drc.footstep_plan_params_t.HORIZONTAL_PLANE;
      obj.footstep_msg.velocity_based_steps = false;
      obj.footstep_msg.goal_pos = drc.position_3d_t();
      obj.footstep_msg.goal_pos.translation = drc.vector_3d_t();
      obj.footstep_msg.goal_pos.rotation = drc.quaternion_t();

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmgl = drake.util.BotLCMGLClient(obj.lc,'drill_planned_path');
      obj.state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.lc.subscribe('EST_ROBOT_STATE', obj.state_monitor);

      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setDebug(true);
      iktraj_options = iktraj_options.setQ(diag(cost(1:getNumPositions(obj.r))));
      iktraj_options = iktraj_options.setQa(diag(vel_cost));
      iktraj_options = iktraj_options.setQv(diag(accel_cost));
      iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumPositions(),1),zeros(obj.r.getNumPositions(),1)); % upper and lower bnd on velocity.
      iktraj_options = iktraj_options.setMajorIterationsLimit(3000);
      iktraj_options = iktraj_options.setMex(true);
      iktraj_options = iktraj_options.setMajorOptimalityTolerance(1e-5);

      obj.ik_options = iktraj_options;
      obj.free_ik_options = iktraj_options.setFixInitialState(false);

      obj.drill_pt_on_hand = drill_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
      obj.drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);

      obj.joint_indices = [back_joint_indices; arm_joint_indices];

      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end


    % Search for an entire trajectory that can hit the drill_points
    % DOES NOT start from q0, just uses this as a seed for the search
    % if free_body_pose is true, also searches for (x,y,z,yaw) within some
    % limits (looking for a place to stand). If it is false, does not allow
    % the pelvis to move
    function [xtraj,snopt_info,infeasible_constraint] = findDrillingMotion(obj, q0, drill_points, free_body_pose, max_delta_x)
      if nargin < 5
        max_delta_x = inf;
      end

      n_points = size(drill_points,2);
      sizecheck(drill_points, [3 n_points]);

      T = 1;
      N = 2*n_points;

      t_vec = linspace(0,T,N);
      t_drill = linspace(0,T,n_points);

      pelvis_body = regexpIndex('pelvis',{obj.r.getBody(:).linkname});

      % x-position constraint in wall frame
      yaw = atan2(obj.drilling_world_axis(2), obj.drilling_world_axis(1));
      HT_wall = [rpy2rotmat([0;0;yaw]) q0(1:3); 0 0 0 1];

      world_pos_x_constraint = WorldPositionInFrameConstraint(obj.r,pelvis_body,zeros(3,1),HT_wall,[-inf;-inf;-inf],[max_delta_x;inf;inf],[-inf inf]);

      % gaze constraint for drill
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);

      posture_constraint = PostureConstraint(obj.r);

      % fix change in x-y-yaw to 0
      body_pose_constraint = PostureChangeConstraint(obj.r,[1; 2; 6], zeros(3,1), zeros(3,1));

      if free_body_pose
        % create posture constraint.  allow moving x, y, z, yaw
        posture_index = setdiff((1:obj.r.num_q)',[1; 2; 6; obj.joint_indices]);
        posture_constraint = posture_constraint.setJointLimits(6, q0(6) - .15, q0(6) + .15);  %arbitrary yaw limit to not turn too much

      else
        posture_index = setdiff((1:obj.r.num_q)',[obj.joint_indices]);
      end
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits(8,-inf,.1);
      posture_constraint = posture_constraint.setJointLimits(9,-.2,.2);


      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end

      % create drill position constraints
      x_drill_traj = PPTrajectory(foh(t_drill,drill_points));
      drill_pos_constraint = cell(1,N);
      for i=1:N,
        drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,...
          x_drill_traj.eval(t_vec(i)),x_drill_traj.eval(t_vec(i)),[t_vec(i) t_vec(i)]);
      end

      q_nom = zeros(obj.r.num_q,N);

      % Find nominal pose
      diff_opt = inf;
      q_end_nom = q0;
      for i=1:50,
        [q_tmp,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0 + .2*randn(obj.r.num_q,1),q0,...
          drill_pos_constraint{1},drill_dir_constraint,posture_constraint,world_pos_x_constraint,obj.ik_options);

        c_tmp = (q_tmp - q0)'*obj.ik_options.Q*(q_tmp - q0);
        if snopt_info_ik <= 3 && c_tmp < diff_opt
          q_end_nom = q_tmp;
          diff_opt = c_tmp;
        end
      end

      q_nom(:,1) = q_end_nom;
%       q_init = q_end_nom;

      for i=2:N,
        j = 1;
        snopt_info_ik(i) = inf;
        while j<10 && snopt_info_ik(i) > 3
          [q_nom(:,i), snopt_info_ik(i),infeasible_constraint_ik] = inverseKin(obj.r,q_nom(:,i-1),q_nom(:,i-1) + .1*randn(obj.r.num_q,1),...
            drill_pos_constraint{i}, drill_dir_constraint, posture_constraint,world_pos_x_constraint, obj.ik_options);
          j = j+1;
        end
        if(snopt_info_ik(i) > 10)
          send_msg = sprintf('snopt_info = %d. The IK fails for t=%f\n', snopt_info_ik(i), t_vec(i));
          send_status(4,0,0,send_msg);
          display(infeasibleConstraintMsg(infeasible_constraint_ik));
          warning(send_msg);
        end
        q_nom(:,i) = q_nom(:,i);
%         q_init = q_nom(:,i);
      end
      qtraj_guess = PPTrajectory(foh(t_vec,q_nom));

      obj.free_ik_options.setMajorOptimalityTolerance(1e-4);
      obj.free_ik_options = obj.free_ik_options.setMajorIterationsLimit(2000);

      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,body_pose_constraint,world_pos_x_constraint,...
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
      end
    end

    % Create a plan starting from q0 to get the drill to x_drill
    % satisfies the drill gaze constraint only for t=T
    function [xtraj,snopt_info,infeasible_constraint] = createInitialReachPlan(obj, q0, x_drill, T, qseed)
      N = 10;
      t_vec = linspace(0,T,N);

      if nargin < 5
        qseed = q0;
      end

      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold,[t_vec(end) t_vec(end)]);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits(8,-inf,.1);
      posture_constraint = posture_constraint.setJointLimits(9,-.2,.2);


      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end
      % create drill position constraint
      drill_pos_constraint = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill,x_drill,[t_vec(end) t_vec(end)]);

      wall_z = [0;0;1] - [0;0;1]'*obj.drilling_world_axis*obj.drilling_world_axis;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z,obj.drilling_world_axis);

      o_T_f = [[obj.drilling_world_axis wall_y wall_z] x_drill; 0 0 0 1];

      drill_distance_constraint = WorldPositionInFrameConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,o_T_f,-inf(3,1),[0;inf;inf]);

      % Find nominal pose
      diff_opt = inf;
      q_end_nom = q0;
      snopt_info_nom = -1;
      for i=1:50,
        [q_tmp,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,qseed + .1*randn(obj.r.num_q,1),q0,...
          drill_pos_constraint,drill_dir_constraint,posture_constraint,obj.ik_options);

        c_tmp = (q_tmp - q0)'*obj.ik_options.Q*(q_tmp - q0);
        if snopt_info_ik <= 3 && c_tmp < diff_opt
          q_end_nom = q_tmp;
          diff_opt = c_tmp;
          snopt_info_nom = snopt_info_ik;
          infeasible_constraint = infeasible_constraint_ik;
        end
      end
      snopt_info = snopt_info_nom;

      if(diff_opt == inf)
        send_msg = sprintf('snopt_info = %d. The IK fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));


      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint,drill_dir_constraint,posture_constraint,drill_distance_constraint,obj.ik_options);

      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end

      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
%         xtraj = obj.posture_pub.generateAndPublishPosturePlan(q0,q_end_nom,0);
%         msg = bot_core.joint_angles_t;
%         msg.robot_name = 'atlas';
%         msg.num_joints = length(obj.joint_indices);
%         msg.utime = etime(clock,[1970 1 1 0 0 0])*1e6;
%         msg.joint_name = obj.r.getStateFrame.coordinates(obj.joint_indices);
%         msg.joint_position = q_end_nom(obj.joint_indices);
%         obj.lc.publish('POSTURE_GOAL',msg);
%         xtraj = q_end_nom;
      end

      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end

    % Create a plan from q0 to get the drill to x_drill_final
    % satisfies the drill gaze constraint for all T
    function [xtraj,snopt_info,infeasible_constraint] = createDrillingPlan(obj, q0, x_drill_final, T)
      N = 10;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);

      t_vec = linspace(0,T,N);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits(8,-inf,.1);
      posture_constraint = posture_constraint.setJointLimits(9,-.2,.2);

      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end

      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);
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

    function [xtraj,snopt_info,infeasible_constraint] = createCircularPlan(obj, q0, x_drill_center, arc, speed)
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);

      radius_vec = -x_drill_center + x_drill_init;
      radius_vec = radius_vec - radius_vec'*obj.drilling_world_axis*obj.drilling_world_axis;
      radius = norm(radius_vec);

      %move x_drill_center to align with normal
      x_drill_center = x_drill_init - radius_vec;

      T = arc*radius/speed;

      N = min(4,ceil(2*T));

      t_vec = linspace(0,T,N);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits(8,-inf,.1);
      posture_constraint = posture_constraint.setJointLimits(9,-.2,.2);

      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end

      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);
      % create drill position constraints
%       x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      wall_z = [0;0;1] - [0;0;1]'*obj.drilling_world_axis*obj.drilling_world_axis;
      wall_z = wall_z/norm(wall_z);
      wall_y = cross(wall_z,obj.drilling_world_axis);
      theta = linspace(0,arc,N) + atan2(radius_vec'*wall_z,radius_vec'*wall_y);
      x_drill = repmat(x_drill_center,1,N) + radius*wall_z*sin(theta) + radius*wall_y*cos(theta);

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

      obj.ik_options = obj.ik_options.setMajorIterationsLimit(200);
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

    function obj = updateWallNormal(obj, normal)
      obj.drilling_world_axis = normal/norm(normal);
    end

    function obj = updateDrill(obj, drill_pt_on_hand, drill_axis_on_hand)
      obj.drill_pt_on_hand = drill_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
    end


    % Create a plan from x_drill_init to x_drill_final
    % DOES NOT start from q0, just uses this as a seed for the search
    % function is for debugging and testing potential cuts
    function [xtraj,snopt_info,infeasible_constraint] = createLinePlan(obj, q0, x_drill_init, x_drill_final, T)
      N = 10;
      t_vec = linspace(0,T,N);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));

      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end

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

    % Simple joint plan from q0 to qf
    % no idea why i'm using IK for this.  probably shouldn't
    function [xtraj, snopt_info, infeasible_constraint] = createGotoPlan(obj,q0,qf,T)
      N = 5;
      t_vec = linspace(0,T,N);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));

      final_posture_constraint = PostureConstraint(obj.r, [T T]);
      final_posture_constraint = final_posture_constraint.setJointLimits(obj.joint_indices, qf(obj.joint_indices), qf(obj.joint_indices));

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

    % publish trajectory as a plan
    % also draw the drill tip with LCMGL
    function publishTraj(obj,xtraj,snopt_info)
      utime = etime(clock,[1970 1 1 0 0 0])*1e6;
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
%       xtraj_atlas(1,:) = ones(1,length(ts)); % make everything a keyframe
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

    % not currently used
    % publish a trajectory as a pose sequence, but it isn't interpreted as
    % expected by the viewer
    function publishPoseTraj(obj,xtraj)
      utime = etime(clock,[1970 1 1 0 0 0])*1e6;
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2*nq_atlas,length(ts));
      xtraj_atlas((1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
      for i=1:length(ts),
        obj.pose_pub.publish(xtraj_atlas(:,i),utime);
      end
    end

    % publish a walking goal
    % pose is [x;y;z;quat]
    function publishWalkingGoal(obj,pose)
      utime = obj.state_monitor.getLastTimestamp();
      obj.footstep_msg.utime = utime;
      obj.footstep_msg.goal_pos.translation.x = pose(1);
      obj.footstep_msg.goal_pos.translation.y = pose(2);
      obj.footstep_msg.goal_pos.translation.z = 0;%pose(3);
      rpy = quat2rpy(pose(4:7));
      pose(4:7) = rpy2quat([0;0;rpy(3)]);
      obj.footstep_msg.goal_pos.rotation.w = pose(4);
      obj.footstep_msg.goal_pos.rotation.x = pose(5);
      obj.footstep_msg.goal_pos.rotation.y = pose(6);
      obj.footstep_msg.goal_pos.rotation.z = pose(7);

      obj.lc.publish('WALKING_GOAL', obj.footstep_msg);
    end

  end
end