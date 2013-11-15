classdef drillButtonPlanner
  %NOTEST
  
  
  properties
    r
    atlas
    plan_pub
    v
    button_pt_on_hand
    button_axis_on_hand
    button_hand_body
    drill_axis_on_hand
    
    finger_pt_on_hand
    finger_axis_on_hand
    finger_hand_body
    
    finger_joint_indices
    button_joint_indices
    back_joint_indices
    
    ik_options
    doVisualization;
    doPublish;
    default_axis_threshold = 20*pi/180;
    atlas2robotFrameIndMap
    lc
    lcmgl
  end
  
  methods
    function obj = drillButtonPlanner(r,atlas,button_pt_on_hand, button_axis_on_hand, drill_axis_on_hand,...
        finger_pt_on_hand, finger_axis_on_hand, buttonInRightHand, doVisualization, doPublish)
      obj.atlas = atlas;
      obj.r = r;
      obj.doVisualization = doVisualization;
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
        obj.v.playback_speed = 5;
      end
      obj.button_pt_on_hand = button_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand;
      obj.button_axis_on_hand = button_axis_on_hand;
      obj.finger_pt_on_hand = finger_pt_on_hand;
      obj.finger_axis_on_hand = finger_axis_on_hand;
      obj.doPublish = doPublish;
      
      joint_names = obj.atlas.getStateFrame.coordinates(1:getNumDOF(obj.atlas));
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmgl = drake.util.BotLCMGLClient(obj.lc,'button_planned_path');
      
      if obj.doPublish
        obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      end
            
      if buttonInRightHand
        obj.button_hand_body = regexpIndex('r_hand',{r.getBody(:).linkname});
        obj.finger_hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
        obj.finger_joint_indices = regexpIndex('^l_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
        obj.button_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      else
        obj.button_hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
        obj.finger_hand_body = regexpIndex('l_hand',{r.getBody(:).linkname});
        obj.finger_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
        obj.button_joint_indices = regexpIndex('^l_arm_[a-z]*[x-z]$',r.getStateFrame.coordinates);
      end
      
      obj.back_joint_indices = regexpIndex('^back_bk[x-z]$',r.getStateFrame.coordinates);
      
      cost = ones(34,1);
      cost([1 2 6]) = 10*ones(3,1);
      cost(3) = 100;      
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
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createPrePokePlan(obj, q0, T)
      N = 5;
      t_vec = linspace(0,T,N);

      % Get world button axis
      kinsol = obj.r.doKinematics(q0);
      button_state = obj.r.forwardKin(kinsol, obj.button_hand_body, obj.button_pt_on_hand, 2);
      button_axis_world = -quat2rotmat(button_state(4:7))*obj.button_axis_on_hand % flip sign so axis points AT the button
      button_pos = button_state(1:3);
      
      finger_target = button_pos - button_axis_world*.1;
      
      % create drill direction constraint
      finger_dir_constraint = WorldGazeDirConstraint(obj.r,obj.finger_hand_body,obj.finger_axis_on_hand,...
        button_axis_world, obj.default_axis_threshold,[t_vec(end) t_vec(end)]);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',[obj.finger_joint_indices; obj.button_joint_indices]);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
%       posture_constraint = posture_constraint.setJointLimits(8,-inf,.25);
      
      % create drill position constraint
      finger_pos_constraint = WorldPositionConstraint(obj.r,obj.finger_hand_body,...
        obj.finger_pt_on_hand,finger_target,finger_target,[t_vec(end) t_vec(end)]);
      
      % Find nominal pose
      diff_opt = inf;
      q_end_nom = q0;
      for i=1:50,
        [q_tmp,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0 + .1*randn(obj.r.num_q,1),q0,...
          finger_pos_constraint,finger_dir_constraint,posture_constraint,obj.ik_options);
        
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
        finger_pos_constraint,finger_dir_constraint,posture_constraint,obj.ik_options);
      
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
    
    function [xtraj,snopt_info,infeasible_constraint] = createPokePlan(obj, q0, offset, T)
      N = 5;
      t_vec = linspace(0,T,N);
      
      % Get world button axis
      kinsol = obj.r.doKinematics(q0);
      button_state = obj.r.forwardKin(kinsol, obj.button_hand_body, obj.button_pt_on_hand, 2);
      button_axis_world = -quat2rotmat(button_state(4:7))*obj.button_axis_on_hand; % flip sign so axis points AT the button
      drill_axis_world = -quat2rotmat(button_state(4:7))*obj.drill_axis_on_hand; % flip sign so axis points AT the button
      button_pos = button_state(1:3);
      
      
      % in the offset axis, button_axis is X, drill_axis is Z
      offset_y = cross(drill_axis_world, button_axis_world);
      
      finger_target = button_pos + offset(1)*button_axis_world + offset(2)*offset_y + offset(3)*drill_axis_world;
      
      % create drill direction constraint
      finger_dir_constraint = WorldGazeDirConstraint(obj.r,obj.finger_hand_body,obj.finger_axis_on_hand,...
        button_axis_world, obj.default_axis_threshold);
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.finger_joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill position constraint
      finger_pos_constraint = WorldPositionConstraint(obj.r,obj.finger_hand_body,...
        obj.finger_pt_on_hand,finger_target,finger_target,[t_vec(end) t_vec(end)]);
      
      % Find nominal pose
      diff_opt = inf;
      q_end_nom = q0;
      for i=1:50,
        [q_tmp,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0 + .1*randn(obj.r.num_q,1),q0,...
          finger_pos_constraint,finger_dir_constraint,posture_constraint,obj.ik_options);
        
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
        finger_pos_constraint,finger_dir_constraint,posture_constraint,obj.ik_options);
      
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
      N = 3;
      t_vec = linspace(0,T,N);
              arm_joint_indices = regexpIndex('^r_arm_[a-z]*[x-z]$',obj.r.getStateFrame.coordinates);

      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',arm_joint_indices);
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      final_posture_constraint = PostureConstraint(obj.r, [T T]);
      final_posture_constraint = final_posture_constraint.setJointLimits(arm_joint_indices, qf(arm_joint_indices), qf(arm_joint_indices));
      
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
        
        drill_pt = obj.r.forwardKin(kinsol,obj.finger_hand_body,obj.finger_pt_on_hand);
        obj.lcmgl.glVertex3d(drill_pt(1),drill_pt(2),drill_pt(3));
      end
      obj.lcmgl.glEnd();
      obj.lcmgl.switchBuffers();
    end
    
  end
  
end