classdef palmGazePlanner
  %NOTEST
  properties
    r
    atlas
    plan_pub
    v
    gaze_axis_on_l_hand = [0;1;0];
    gaze_axis_on_r_hand = [0;-1;0];
    r_hand_body;
    l_hand_body;
    l_joint_indices;
    r_joint_indices;
    ik_options
    free_ik_options
    doVisualization;
    doPublish;
    default_axis_threshold = 2.5*pi/180;
    atlas2robotFrameIndMap
    allowPelvisHeight
    T_palm_hand_l = inv(HT([0;0.11516;0.015],1.57079,3.14159,3.14159));
    T_palm_hand_r = inv(HT([0;-0.11516;-0.015],1.57079,0,0));
  end
  
  methods    
    function obj = palmGazePlanner(r,atlas, doVisualization, doPublish, allowPelvisHeight)
      obj.atlas = atlas;
      obj.r = r;
      obj.doVisualization = doVisualization;
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
        obj.v.playback_speed = 5;
      end
      
      obj.doPublish = doPublish;
      
      obj.allowPelvisHeight = allowPelvisHeight;
      
      joint_names = obj.atlas.getStateFrame.coordinates(1:getNumDOF(obj.atlas));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      
      obj.doPublish = doPublish;
      
      if obj.doPublish
        obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      end
      
      back_joint_indices = regexpIndex('^back_bk[x-z]$',r.getStateFrame.coordinates);
      obj.r_hand_body = regexpIndex('r_hand',{r.getBody(:).linkname});
      r_arm_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      obj.l_hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
      l_arm_joint_indices = regexpIndex('^l_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      obj.r_joint_indices = [back_joint_indices; r_arm_joint_indices];
      obj.l_joint_indices = [back_joint_indices; l_arm_joint_indices];

      cost = ones(34,1);
      cost([1 2 6]) = 10*ones(3,1);
      cost(3) = 100;
      cost(back_joint_indices) = [100;1000;100];
      
      vel_cost = cost*.05;
      accel_cost = cost*.05;
 
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
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end
  
    % Create a plan from q0 to get the drill to x_drill_final
    % satisfies the drill gaze constraint for all T
    function [xtraj,snopt_info,infeasible_constraint] = createGazePlan(obj, q0, pose_final, speed, useRightHand)
      N = 4;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      
      if useRightHand
        hand_body = obj.r_hand_body;
        hand_axis = obj.gaze_axis_on_r_hand;
        joint_indices = obj.r_joint_indices;
        palm_to_hand = obj.T_palm_hand_r;
      else
        hand_body = obj.l_hand_body;
        hand_axis = obj.gaze_axis_on_l_hand;
        joint_indices = obj.l_joint_indices;
        palm_to_hand = obj.T_palm_hand_l;
      end
      
      hand_init = obj.r.forwardKin(kinsol,hand_body,zeros(3,1));
      
      world_to_palm  = zeros(4,4);
      world_to_palm(1:3,1:3) = quat2rotmat(pose_final(4:7));
      world_to_palm(1:3,4) = pose_final(1:3);
      world_to_palm(4,4) = 1;
      
      world_to_hand = world_to_palm*palm_to_hand;
      world_axis = world_to_hand(1:3,1:3)*hand_axis;
      
      pose_final = [world_to_hand(1:3,4); rotmat2quat(world_to_hand(1:3,1:3))];
      
      T = norm(pose_final(1:3) - hand_init)/speed;
      
      t_vec = linspace(0,T,N);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      posture_constraint = posture_constraint.setJointLimits(8,-inf,.25);
      
      if obj.allowPelvisHeight
        [z_min, z_max] = obj.atlas.getPelvisHeightLimits(q0);
        posture_constraint = posture_constraint.setJointLimits(3,z_min, z_max);
      end
      
      % create drill direction constraint
%       hand_dir_constraint = WorldGazeOrientConstraint(obj.r,hand_body,hand_axis,...
%         pose_final(4:7),pi,obj.default_axis_threshold);
      hand_dir_constraint = WorldGazeDirConstraint(obj.r,hand_body,hand_axis,...
        world_axis,obj.default_axis_threshold);
      % create drill position constraints
      x_hand = repmat(hand_init,1,N) + (pose_final(1:3) - hand_init)*linspace(0,1,N);
      hand_pos_constraint = cell(1,N-1);
      for i=2:N,
        hand_pos_constraint{i-1} = WorldPositionConstraint(obj.r,hand_body,zeros(3,1),x_hand(:,i),x_hand(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % Find nominal pose
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        hand_pos_constraint{end},hand_dir_constraint,posture_constraint,obj.ik_options);
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
      end
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        hand_pos_constraint{:},hand_dir_constraint,posture_constraint,obj.ik_options);
      
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
      utime = now() * 24 * 60 * 60;
      nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
      xtraj_atlas(2+(1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
      snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));
      
      obj.plan_pub.publish(xtraj_atlas,ts,utime,snopt_info_vector);
    end
  end
end
