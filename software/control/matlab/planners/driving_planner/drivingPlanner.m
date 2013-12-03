classdef drivingPlanner
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
    pt_on_hand
    axis_on_hand
    hand_body
    root_body
    steer_center_in_root
    steer_axis_in_root
    steer_zero_vec_in_root
    steer_radius
    ankle_joint
    joint_indices;
    ik_options
    free_ik_options
    doVisualization = true;
    doPublish = false;
    default_axis_threshold = 5*pi/180;
    atlas2robotFrameIndMap
    lc
    state_monitor
    lcmgl
    doAutoCommit
  end
  
  methods    
    function obj = drivingPlanner(r,atlas,pt_on_hand, axis_on_hand, ...
        root_body, steer_center_in_root, steer_axis_in_root, steer_zero_vec_in_root, steer_radius,...
        useRightHand, doVisualization, doPublish, doAutoCommit)
      obj.root_body = root_body;
      obj.steer_center_in_root = steer_center_in_root;
      obj.steer_axis_in_root = steer_axis_in_root;
      obj.steer_zero_vec_in_root = steer_zero_vec_in_root;
      obj.steer_radius = steer_radius;
      valuecheck(steer_zero_vec_in_root'*steer_axis_in_root,0);
      
      obj.atlas = atlas;
      obj.r = r;
      obj.doVisualization = doVisualization;
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
        obj.v.playback_speed = 5;
      end
      
      obj.doAutoCommit = doAutoCommit;
      
      obj.doPublish = doPublish;
      
      joint_names = obj.atlas.getStateFrame.coordinates(1:getNumDOF(obj.atlas));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      
      obj.doPublish = doPublish;
      obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      obj.pose_pub = CandidateRobotPosePublisher('CANDIDATE_ROBOT_ENDPOSE',true,joint_names);
      
      if useRightHand
        obj.hand_body = regexpIndex('r_hand',{r.getBody(:).linkname});
        arm_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      else
        obj.hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
        arm_joint_indices = regexpIndex('^l_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      end
      
      obj.ankle_joint = regexpIndex('^l_leg_aky$',r.getStateFrame.coordinates);

      cost = ones(34,1);
      cost([1 2 6]) = 5000*ones(3,1);
      cost(3) = 200;
      
      vel_cost = cost*.05;
      accel_cost = cost*.05;
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmgl = drake.util.BotLCMGLClient(obj.lc,'drill_planned_path');
      obj.state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.lc.subscribe('EST_ROBOT_STATE', obj.state_monitor);

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
      
      obj.pt_on_hand = pt_on_hand;
      obj.axis_on_hand = axis_on_hand/norm(axis_on_hand);
      
      obj.joint_indices = arm_joint_indices;
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end
     
    function [xtraj,snopt_info,infeasible_constraint] = createDrivingPlan(obj, q0, steering_angle, ankle_angle, steering_speed, ankle_speed)
      % get current steering angle
      kinsol = obj.r.doKinematics(q0);
      x_pt_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.pt_on_hand);
      x_root_init = obj.r.forwardKin(kinsol,obj.root_body, zeros(3,1),2);
      R_root = quat2rotmat(x_root_init(4:7));
      x_root_init = x_root_init(1:3);
      
      x_init_in_root = R_root'*(x_pt_init - x_root_init);
      
      radius_vec = x_init_in_root - obj.steer_center_in_root;
      radius_vec = radius_vec - radius_vec'*obj.steer_axis_in_root*obj.steer_axis_in_root;
      radius = norm(radius_vec);
      
      steering_init = acos(radius_vec'/radius*obj.steer_zero_vec_in_root);
      
      % create ankle and angle trajectories
      ankle_init = q0(obj.ankle_joint);
      T_ankle = abs((ankle_angle - ankle_init)/ankle_speed);
      T_steering = abs((steering_init - steering_angle)/steering_speed);
      T = max(T_ankle, T_steering);
      
      N = 10;
      if T_ankle > T_steering
        t_vec = linspace(0,T_ankle,N);
        ankle_vec = linspace(ankle_init, ankle_angle, N);
        N_steering = ceil(N*T_steering/T_ankle);
        steering_vec_1 = linspace(steering_init, steering_angle, N_steering);
        steering_vec_2 = linspace(steering_angle, steering_angle, N - N_steering);
        steering_vec = [steering_vec_1 steering_vec_2];
      else
        t_vec = linspace(0,T_steering,N);
        steering_vec = linspace(steering_init, steering_angle, N);
        N_ankle = ceil(N*T_ankle/T_steering);
        ankle_vec_1 = linspace(ankle_init, ankle_angle, N_ankle);
        ankle_vec_2 = linspace(ankle_angle, ankle_angle, N - N_ankle);
        ankle_vec = [ankle_vec_1 ankle_vec_2];
      end
      
      % generate ankle and steering constraints
      steering_axis_1 = obj.steer_zero_vec_in_root;
      steering_axis_2 = cross(steering_axis_1, obj.steer_axis_in_root);
      
      ankle_constraint = cell(1,N);
      steering_constraint = cell(1,N);
      for i=1:N,
        ankle_constraint{i} = PostureConstraint(obj.r,[t_vec(i) t_vec(i)]);
        ankle_constraint{i} = ankle_constraint{i}.setJointLimits(obj.ankle_joint, ankle_vec(i), ankle_vec(i));
        
        x_steering_in_root = obj.steer_center_in_root + obj.steer_radius*(cos(steering_vec(i))*steering_axis_1 + sin(steering_vec(i))*steering_axis_2);
        x_steering_in_world = R_root*(x_steering_in_root - x_root_init) + x_root_init;
        steering_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.pt_on_hand,x_steering_in_world,x_steering_in_world,[t_vec(i) t_vec(i)]);
      end
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',[obj.ankle_joint; obj.joint_indices]);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      steering_axis_world = R_root*obj.steer_axis_in_root;
            
      % create drill direction constraint
      hand_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.axis_on_hand,...
        steering_axis_world,obj.default_axis_threshold);
      
      % Find nominal pose
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        steering_pos_constraint{end}, ankle_constraint{end},hand_dir_constraint,posture_constraint,obj.ik_options);
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
        steering_pos_constraint{:}, ankle_constraint{:},hand_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info <= 10
        obj.publishTraj(xtraj,snopt_info);
      end
      
      %todo...execute plan immediately, haha!
    end
    
    function obj = updateWallNormal(obj, normal)
      obj.drilling_world_axis = normal/norm(normal);
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
%           [obj.pt_on_hand, obj.pt_on_hand + .0254*obj.axis_on_hand]);
%         obj.lcmgl.line3(drill_pts(1,1),drill_pts(2,1),drill_pts(3,1),...
%           drill_pts(1,2),drill_pts(2,2),drill_pts(3,2));
        
        drill_pt = obj.r.forwardKin(kinsol,obj.hand_body,obj.pt_on_hand);
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
      obj.footstep_msg.goal_pos.translation.z = pose(3);
      obj.footstep_msg.goal_pos.rotation.w = pose(4);
      obj.footstep_msg.goal_pos.rotation.x = pose(5);
      obj.footstep_msg.goal_pos.rotation.y = pose(6);
      obj.footstep_msg.goal_pos.rotation.z = pose(7);
      
      obj.lc.publish('WALKING_GOAL', obj.footstep_msg);      
    end
    
  end
end