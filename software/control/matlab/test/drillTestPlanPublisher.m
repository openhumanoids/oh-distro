classdef drillTestPlanPublisher
  %NOTEST
  properties
    r
    atlas
    plan_pub
    v
    drill_pt_on_hand
    drill_axis_on_hand
    hand_body = 29;
    joint_indices = [22:26 33];
    ik_options
    drilling_world_axis
    doVisualization = true;
    doPublish = false;
    default_axis_threshold = .2*pi/180;
    atlas2robotFrameIndMap
  end
  
  methods
    function obj = drillTestPlanPublisher(drill_pt_on_hand, drill_axis_on_hand, drilling_world_axis)
      obj.atlas = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
      obj.r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true));
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
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
      iktraj_options = iktraj_options.setMajorIterationsLimit(300);
      iktraj_options = iktraj_options.setMex(false);
      
      obj.ik_options = iktraj_options;
      
      obj.drill_pt_on_hand = drill_pt_on_hand;
      obj.drill_axis_on_hand = drill_axis_on_hand/norm(drill_axis_on_hand);
      obj.drilling_world_axis = drilling_world_axis/norm(drilling_world_axis);
      
      for i = 1:obj.atlas.getNumStates
        obj.atlas2robotFrameIndMap(i) = find(strcmp(obj.atlas.getStateFrame.coordinates{i},obj.r.getStateFrame.coordinates));
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createInitialReachPlan(obj, q0, x_drill, T)
      N = 5;
      t_vec = linspace(0,T,N);
      qtraj_guess = PPTrajectory(foh([0 1],[q0, q0]));
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold,[t_vec(end) t_vec(end)]);
      
      % create drill position constraint
      drill_pos_constraint = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill,x_drill,[t_vec(end) t_vec(end)]);
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTrajWcollision(obj.r,0,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint,drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info == 1
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info == 1
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createDrillingPlan(obj, q0, x_drill_final, T)
      N = 10;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
      
      t_vec = linspace(0,T,N);
      qtraj_guess = PPTrajectory(foh([0 1],[q0, q0+.1*randn(length(q0),1)]));
      
      % create posture constraint
      posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
      
      % create drill direction constraint
      drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
        obj.drilling_world_axis,obj.default_axis_threshold);
      
      % create drill position constraints
      x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
      drill_pos_constraint = cell(1,N-1);
      for i=2:N,
        drill_pos_constraint{i-1} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
      end
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTrajWcollision(obj.r,0,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info == 1
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info == 1
        obj.publishTraj(xtraj,snopt_info);
      end
    end
    
    function [xtraj,snopt_info,infeasible_constraint] = createCircleCutPlan(obj, q0, circle_center, T)
      N = 10;
      %evaluate current drill location
      kinsol = obj.r.doKinematics(q0);
      x_drill_init = obj.r.forwardKin(kinsol,obj.hand_body,obj.drill_pt_on_hand);
      
      t_vec = linspace(0,T,N);
      qtraj_guess = PPTrajectory(foh([0 1],[q0, q0+.1*randn(length(q0),1)]));
      
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
      
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTrajWcollision(obj.r,0,...
        t_vec,qtraj_guess,qtraj_guess,...
        drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.ik_options);
      
      if(snopt_info > 10)
        send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
        send_status(4,0,0,send_msg);
        display(infeasibleConstraintMsg(infeasible_constraint));
        warning(send_msg);
      end
      
      if obj.doVisualization && snopt_info == 1
        obj.v.playback(xtraj);
      end
      
      if obj.doPublish && snopt_info == 1
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
    end
    
  end
end