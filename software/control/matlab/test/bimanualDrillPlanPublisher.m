classdef bimanualDrillPlanPublisher
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
    v
    grasp_pt_on_left_hand
    lh_on_hand
    lh_grasp_axis
    drill_pt_on_hand
    drill_axis_on_hand
    hand_body = 29;
    left_hand_body = 17;
    joint_indices = [7:14 21 22:26 33];
    ik_options
    free_ik_options
    drilling_world_axis
    doVisualization = true;
    doPublish = false;
    default_axis_threshold = 2*pi/180;
    atlas2robotFrameIndMap
  end
  
  methods
    function obj = bimanualDrillPlanPublisher(r,atlas,drill_pt_on_hand, drill_axis_on_hand, ...
      drilling_world_axis, lh_on_hand,lh_grasp_axis, doVisualization, doPublish)
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
      joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
      obj.plan_pub = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN',true,joint_names);
      cost = ones(34,1);
      
      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setDebug(true);
      iktraj_options = iktraj_options.setQ(diag(cost(1:getNumDOF(obj.r))));
      iktraj_options = iktraj_options.setQa(0.05*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setQv(0*eye(getNumDOF(obj.r)));
      iktraj_options = iktraj_options.setqdf(zeros(obj.r.getNumDOF(),1),zeros(obj.r.getNumDOF(),1)); % upper and lower bnd on velocity.
      iktraj_options = iktraj_options.setMajorIterationsLimit(50);
      iktraj_options = iktraj_options.setMex(false);
      
      obj.ik_options = iktraj_options;
      obj.free_ik_options = iktraj_options.setFixInitialState(false);
      
      obj.drill_pt_on_hand = drill_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
      obj.drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
      obj.lh_on_hand = lh_on_hand;
      obj.lh_grasp_axis = lh_grasp_axis/norm(lh_grasp_axis);
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
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
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0 + .1*randn(obj.r.num_q,1),q0,...
        drill_pos_constraint,drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
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
%     
%     function [xtraj,snopt_info,infeasible_constraint] = createDrillingPlan(obj, q0, x_drill_final, first_cut_dir, T)
%       N = 10;
%       %evaluate current drill location
%       kinsol = obj.r.doKinematics(q0);
%       x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
%       
%       t_vec = linspace(0,T,N);
%       qtraj_guess = PPTrajectory(foh([0 1],[q0, q0+.1*randn(length(q0),1)]));
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
%       posture_constraint = PostureConstraint(obj.r);
%       posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
% 
%       if ~isempty(first_cut_dir)
%         % generate desired quaternion
%         R_hand = [obj.drill_axis_on_hand obj.drill_dir_des cross(obj.drill_axis_on_hand, obj.drill_dir_des)];
%         R_world = [obj.drilling_world_axis first_cut_dir cross(obj.drilling_world_axis, first_cut_dir)];
%         R_rel = R_world*R_hand';
%         quat_des = rotmat2quat(R_rel);
%         
%         % create drill direction constraint
%         drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%           quat_des, obj.default_axis_threshold,obj.drill_dir_threshold);
%       else
%         % create drill direction constraint
%         drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%           obj.drilling_world_axis,obj.default_axis_threshold);
%       end
%       
%       % create drill position constraints
%       x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
%       drill_pos_constraint = cell(1,N-1);
%       for i=2:N,
%         drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
%       end
%       
%       [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
%         t_vec,qtraj_guess,qtraj_guess,...
%         drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info > 10)
%         send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint));
%         warning(send_msg);
%       end
%       
%       if obj.doVisualization && snopt_info <= 10
%         obj.v.playback(xtraj);
%       end
%       
%       if obj.doPublish && snopt_info <= 10
%         obj.publishTraj(xtraj,snopt_info);
%       end
%     end
%     
%     function [xtraj,snopt_info,infeasible_constraint] = createCircleCutPlan(obj, q0, circle_center, T)
%       N = 10;
%       %evaluate current drill location
%       kinsol = obj.r.doKinematics(q0);
%       x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
%       
%       t_vec = linspace(0,T,N);
%       qtraj_guess = PPTrajectory(foh([0 1],[q0, q0+.1*randn(length(q0),1)]));
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
%       posture_constraint = PostureConstraint(obj.r);
%       posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
%       
%       % create drill direction constraint
%       drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%         obj.drilling_world_axis,obj.default_axis_threshold);
%       
%       % create drill position constraints
%       rot_axis = obj.drilling_world_axis;
%       radius_vec = x_drill_init - circle_center;
%       radius_vec = radius_vec - rot_axis*(rot_axis'*radius_vec);
%       radius = norm(radius_vec);
%       theta = linspace(0,2*pi,N);
%       
%       rot_axis*rot_axis'*radius_vec*(1-cos(theta)) + radius_vec*cos(theta) + cross(rot_axis,radius_vec)*sin(theta);
%       
%       x_drill = repmat(x_drill_init - radius_vec,1,N) + ...
%         rot_axis*rot_axis'*radius_vec*(1-cos(theta)) + radius_vec*cos(theta) + cross(rot_axis,radius_vec)*sin(theta);
%       drill_pos_constraint = cell(1,N-1);
%       for i=2:N,
%         drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
%       end
%       
%       [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
%         t_vec,qtraj_guess,qtraj_guess,...
%         drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info > 10)
%         send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint));
%         warning(send_msg);
%       end
%       
%       if obj.doVisualization && snopt_info <= 10
%         obj.v.playback(xtraj);
%       end
%       
%       if obj.doPublish && snopt_info <= 10
%         obj.publishTraj(xtraj,snopt_info);
%       end
%     end
%     
    function [xtraj,snopt_info,infeasible_constraint] = createLinePlan(obj, q0, x_drill_init, x_drill_final, T)
      N = 4;
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
      
      % create left hand constraints
      lh_orient_constraint = RelativeFixedQuatConstraint(obj.r, obj.hand_body, obj.left_hand_body);
      lh_pos_constraint = TwoBodyRelativePositionConstraint(obj.r, obj.hand_body, obj.left_hand_body, [obj.lh_on_hand zeros(3,1)], zeros(3,1), zeros(3,1));
      lh_gaze_constraint = WorldGazeDirConstraint(obj.r,obj.left_hand_body,obj.lh_grasp_axis,...
        [0;1;0],obj.default_axis_threshold,[0 0]);
      
      % Find nominal poses
      [q_start_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
        drill_pos_constraint{1},drill_dir_constraint,posture_constraint,lh_pos_constraint,lh_gaze_constraint,obj.ik_options);
      
      if(snopt_info_ik > 10)
        send_msg = sprintf('snopt_info = %d. The IK (init) fails.',snopt_info_ik);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint_ik));
        warning(send_msg);
        xtraj = [];
        snopt_info = snopt_info_ik;
        infeasible_constraint = infeasible_constraint_ik;
        return;
      end      
      
      
      qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_start_nom]));
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,...
        lh_pos_constraint, lh_orient_constraint, lh_gaze_constraint,obj.free_ik_options);
      
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
%     
%     
%     function [xtraj,snopt_info,infeasible_constraint] = createDirectedLinePlan(obj, q0, x_drill_final, T)
%       N = 10;
%       t_vec = linspace(0,T,N);
%       
%       kinsol = obj.r.doKinematics(q0);
%       x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
%       
%       % find direction of drill motion
%       drill_motion_dir = (x_drill_final - x_drill_init);
%       drill_motion_dir = drill_motion_dir/norm(drill_motion_dir);
%       valuecheck(drill_motion_dir'*obj.drilling_world_axis,0,1e-4);
%       
%       
%       % generate desired quaternion
%       R_hand = [obj.drill_axis_on_hand obj.drill_dir_des cross(obj.drill_axis_on_hand, obj.drill_dir_des)];
%       R_world = [obj.drilling_world_axis drill_motion_dir cross(obj.drilling_world_axis, drill_motion_dir)];
%       R_rel = R_world*R_hand';
%       quat_des = rotmat2quat(R_rel);
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',[obj.joint_indices]');
%       posture_constraint = PostureConstraint(obj.r);
%       posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
%       
%       % create drill direction constraint
%       drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%         quat_des, obj.default_axis_threshold,obj.drill_dir_threshold);
%       
%       % create drill position constraints
%       x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
%       drill_pos_constraint = cell(1,N-1);
%       for i=2:N,
%         drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
%       end
%       
%       % Find nominal poses
%       [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
%         drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info_ik > 10)
%         send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint_ik));
%         warning(send_msg);
%       end
%       
%       qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end_nom]));
%       
%       [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
%         t_vec,qtraj_guess,qtraj_guess,...
%         drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info > 10)
%         send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint));
%         warning(send_msg);
%       end
%       
%       if obj.doVisualization && snopt_info <= 10
%         obj.v.playback(xtraj);
%       end
%       
%       if obj.doPublish && snopt_info <= 10
%         obj.publishTraj(xtraj,snopt_info);
%       end
%     end
%     
%     function [xtraj,snopt_info,infeasible_constraint] = createConstrainedLinePlan(obj, q0, x_drill_init, x_drill_final, quat_des, threshold, T)
%       N = 10;
%       t_vec = linspace(0,T,N);
%       
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',[1 2 3 6 obj.joint_indices]');
%       posture_constraint_free_base = PostureConstraint(obj.r);
%       posture_constraint_free_base = posture_constraint_free_base.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
%       
%       % create drill direction constraint
% %       WorldGazeOrientConstraint(obj.r,obj.hand_body,axis,quat_des,conethreshold,threshold,tspan)
%       drill_dir_constraint = WorldGazeOrientConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%         quat_des, obj.default_axis_threshold,threshold);
% %         obj.drilling_world_axis,obj.default_axis_threshold);
% 
%       body_pose_constraint = WorldFixedBodyPoseConstraint(obj.r,2); % fix the pelvis
%       
%       % create drill position constraints
%       x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
%       drill_pos_constraint = cell(1,N);
%       for i=1:N,
%         drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
%       end
%       
%       % Find nominal poses
%       [q_start_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
%         drill_pos_constraint{1},drill_dir_constraint,posture_constraint_free_base,obj.ik_options);
%       
%       if(snopt_info_ik > 10)
%         send_msg = sprintf('snopt_info = %d. The IK (start) fails.',snopt_info_ik);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint_ik));
%         warning(send_msg);
%       end
%       
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
%       posture_constraint = PostureConstraint(obj.r);
%       posture_constraint = posture_constraint.setJointLimits(posture_index,q_start_nom(posture_index),q_start_nom(posture_index));
%       
% %       posture_constraint = posture_constraint_free_base;
%       
%       % Find nominal poses
%       [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
%         drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info_ik > 10)
%         send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint_ik));
%         warning(send_msg);
%       end
%       
%       
%       qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
%       
%       [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
%         t_vec,qtraj_guess,qtraj_guess,body_pose_constraint,...
%         drill_pos_constraint{:},drill_dir_constraint,posture_constraint_free_base,obj.free_ik_options);
%       
%       if(snopt_info > 10)
%         send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint));
%         warning(send_msg);
%       end
%       
%       if obj.doVisualization && snopt_info <= 10
%         obj.v.playback(xtraj);
%       end
%       
%       if obj.doPublish && snopt_info <= 10
%         obj.publishTraj(xtraj,snopt_info);
%       end
%     end
%     
%     function publishTraj(obj,xtraj,snopt_info)
%       utime = now() * 24 * 60 * 60;   
%       nq_atlas = length(obj.atlas2robotFrameIndMap)/2;
%       ts = xtraj.pp.breaks;
%       q = xtraj.eval(ts);
%       xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
%       xtraj_atlas(2+(1:nq_atlas),:) = q(obj.atlas2robotFrameIndMap(1:nq_atlas),:);
%       snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));
%       
%       obj.plan_pub.publish(xtraj_atlas,ts,utime,snopt_info_vector);
%     end
%     
  end
end