classdef drivingPlanner

  properties
    T_wheel_2_world
    turn_radius
    r
    options
    R_hand
    hand
    nq
    arm_idx
    non_arm_idx
    ik_options
    q0
    hand_pt
    final_posture_constraint
    posture_constraint
    steering_position_constraint
    hand_orientation_constraint
    N
    qd_max
    qstar
    data;
    plan_publisher
    lc
    state_monitor
    q_touch % stores the latest touch, so we can recenter the steering wheel
    lwy_idx
  end

  methods
    function obj = drivingPlanner(r,T,q0,options)
      if nargin < 3
        options = struct();
      end
      obj.q0 = q0;
      obj.r = r;
      obj.options = options;
      obj.hand = 'l_hand';
      obj.hand_pt = [0;-0.25;0];
      obj.T_wheel_2_world = T;
      obj.N = 20;

      if obj.hand(1) == 'l'
        obj.arm_idx = obj.r.findPositionIndices('l_arm');
      else
        obj.arm_idx = obj.r.findPositionIndices('r_arm');
      end

      if ~isfield(options,'wheel_radius') obj.options.wheel_radius = 0.2; end
      if ~isfield(options,'turn_radius'), obj.options.turn_radius = obj.options.wheel_radius; end
      if ~isfield(options,'reach_depth'), obj.options.reach_depth = 0; end
      if ~isfield(options,'quat_tol'), obj.options.quat_tol = 0.02; end
      if ~isfield(options,'tol'), obj.options.tol = 0.0; end

      % need to allow to specify the turn radius

      obj.nq = r.getNumPositions;
      obj.non_arm_idx = setdiff([1:obj.nq],obj.arm_idx);
      S = load(r.fixed_point_file);
      obj.qstar = S.xstar(1:obj.nq);
      obj.data = load([getenv('DRC_BASE'),'/software/control/matlab/planners/driving_planner/data.mat']);


      % options.R should be the hand rotation, specified in rpy from the frame aligned with
      % the steering wheel
      % reach_depth

      iktraj_options = IKoptions(obj.r);
      iktraj_options = iktraj_options.setDebug(true);
      Q = eye(obj.nq);
      Q(1:6) = 0;
      iktraj_options = iktraj_options.setQ(Q);
      iktraj_options = iktraj_options.setMajorIterationsLimit(3000);
      iktraj_options = iktraj_options.setMex(true);
      iktraj_options = iktraj_options.setMajorOptimalityTolerance(1e-5);
      obj.ik_options = iktraj_options;

      %% plan publisher
      obj.plan_publisher = RobotPlanPublisherWKeyFrames('CANDIDATE_MANIP_PLAN', true, obj.r.getManipulator.getPositionFrame.coordinates);

      % Get a listener for robot state
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.state_monitor = drake.util.MessageMonitor(drc.robot_state_t, 'utime');
      obj.lc.subscribe('EST_ROBOT_STATE', obj.state_monitor);
      obj.lwy_idx = obj.r.findPositionIndices('l_arm_lwy');
      obj = obj.setqdmax();
    end

    function [qtraj,q_safe] = planSafe(obj,options,q0)

      if nargin < 3
        q0 = obj.getRobotState();
      end

      q_safe = obj.data.arm_safe;
      q_safe(obj.non_arm_idx) = q0(obj.non_arm_idx);
      qtraj = constructAndRescaleTrajectory([q0,q_safe],obj.qd_max);
      obj.publishTraj(qtraj,1);
    end

    function [qtraj,q_pre_grasp] = planPreGrasp(obj,options,q0)
      if nargin < 2
        options = struct();
      end

      if nargin < 3
        q0 = obj.getRobotState();
      end

      % need to specify reach depth and orientation
      if ~isfield(options,'depth'), options.depth = 0.1; end
      if ~isfield(options,'angle') options.angle = 0; end
      if ~isfield(options,'speed') options.speed = 1; end
      obj.q0 = q0;

      xyz_des = [0;options.depth;0];
      if isfield(options,'xyz_des')
        xyz_des = options.xyz_des;
      end
      lb = xyz_des - obj.options.tol*ones(3,1);
      ub = xyz_des + obj.options.tol*ones(3,1);

      obj = obj.generateConstraints(q0, options);
      q_nom = obj.data.pre_grasp_2;
      position_constraint = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,obj.T_wheel_2_world,lb,ub);
      constraints = {obj.posture_constraint,obj.hand_orientation_constraint, position_constraint};
      [q_pre_grasp,info,infeasible_constraint] = obj.r.inverseKin(q_nom,q_nom,constraints{:},obj.ik_options);
      info
      infeasible_constraint
      q_vals = [q0,q_pre_grasp];
      qtraj = constructAndRescaleTrajectory(q_vals, options.speed*obj.qd_max);
      obj.publishTraj(qtraj,info);
    end

    function [qtraj, q_touch, obj] = planTouch(obj,options,q0)
      if nargin < 2
        options = struct();
      end
      if nargin < 3
        q0 = obj.getRobotState();
      end
      if ~isfield(options,'depth'), options.depth = 0; end
      if ~isfield(options,'speed'), options.speed = 1; end

      xyz_des = [0;options.depth;0];
      if isfield(options,'xyz_des')
        xyz_des = options.xyz_des;
      end
      lb = xyz_des - obj.options.tol*ones(3,1);
      ub = xyz_des + obj.options.tol*ones(3,1);

      % plan to a safe pregrasp
      % plan to reach with the desired orientation
      obj = obj.generateConstraints(q0, options);
      hand_quat_constraint = obj.genHandQuatConstraintFromPose(q0);
      position_constraint = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,obj.T_wheel_2_world,lb,ub);
      constraints = {obj.posture_constraint,hand_quat_constraint, position_constraint};
      q_nom = q0;
      [q_touch,info,infeasible_constraint] = obj.r.inverseKin(q_nom,q_nom,constraints{:},obj.ik_options);
      info
      infeasible_constraint
      obj.q_touch = q_touch;
      q_vals = [q0,q_touch];
      qtraj = constructAndRescaleTrajectory(q_vals, obj.qd_max);
      obj.publishTraj(qtraj,info);
    end

    function [qtraj, q_retract] = planRetract(obj,options,q0)
      if nargin < 2
        options = struct();
      end

      if nargin < 3
        q0 = obj.getRobotState();
      end

      if ~isfield(options,'depth') options.depth = 0.2; end
      if ~isfield(options, 'retract') options.speed = 1; end

      obj = obj.generateConstraints(q0,options);
      xyz_des = [0;options.depth;0];
      lb = xyz_des - obj.options.tol*ones(3,1);
      ub = xyz_des + obj.options.tol*ones(3,1); 
      position_constraint = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,obj.T_wheel_2_world,lb,ub);
      hand_quat_constraint = obj.genHandQuatConstraintFromPose(q0);
      constraints = {position_constraint,hand_quat_constraint,obj.posture_constraint};

      q_nom = obj.data.pre_grasp_2;
      [q_retract,info,infeasible_constraint] = obj.r.inverseKin(q_nom,q_nom,constraints{:},obj.ik_options);
      info
      infeasible_constraint
      q_vals = [q0,q_retract];
      qtraj = constructAndRescaleTrajectory(q_vals, options.speed*obj.qd_max);
      obj.publishTraj(qtraj,info);
    end

    % just use lwy to do the turn
    function [qtraj,q_turn] = planTurn(obj, options, q0)
      if nargin < 2
        options = struct();
      end
      if nargin < 3
        q0 = obj.getRobotState();
      end
      if ~isfield(options,'turn_angle'), options.turn_angle = 0; end
      if ~isfield(options, 'angle_from_touch'), options.angle_from_touch = 0; end
      if ~isfield(options,'use_raw_angle'), options.use_raw_angle = 0; end
      if ~isfield(options,'reset_to_touch'), options.reset_to_touch = 0; end
      if ~isfield(options,'speed') options.speed = 1; end
      if ~isfield(options,'scaling_factor'), options.acceleration_parameter = 2; end
      if ~isfield(options,'t_acc'), options.t_acc = 0.4; end

      lwy_idx = obj.r.findPositionIndices('l_arm_lwy');
      lwy_0 = q0(lwy_idx);
      [jl_min,jl_max] = obj.r.getJointLimits();

      % check to see if we would run into a joint limit, and don't
      info = 1;
      if options.reset_to_touch
        lwy_des = obj.q_touch(lwy_idx);
      elseif options.use_raw_angle
        lwy_des = options.turn_angle;        
      else
        if options.angle_from_touch
          lwy_des = obj.q_touch(lwy_idx) + options.turn_angle;
        else
          lwy_des = q0(lwy_idx) + options.turn_angle;
        end
        % make sure we aren't exceeding joint limits
        if lwy_des > jl_max(lwy_idx)
          disp('Would have hit joint limit, adjusting turn angle')
          lwy_des = jl_max(lwy_idx);
          info = 2;
        elseif lwy_des < jl_min(lwy_idx)
          disp('Would have hit joint limit, adjusting turn angle')
          lwy_des = jl_min(lwy_idx);
          info = 2;
        end
      end

      q_turn = q0;
      q_turn(lwy_idx) = lwy_des;
      varargin = {options.acceleration_parameter,options.t_acc};
      qtraj = constructAndRescaleTrajectory([q0,q_turn],options.speed*obj.qd_max,0,varargin);
      obj.publishTraj(qtraj,info);
    end


    function xtraj = createSteeringPlan(obj,q0)
      % a call to inverseKinTraj
      obj = obj.generateConstraints(q0);
      constraints = [{obj.posture_constraint,obj.hand_orientation_constraint},obj.steering_position_constraint];
      ts = [1:obj.N];
      [xtraj,info,infeasible_constraint] = inverseKinTraj(obj.r,t,q_seed_traj,q_nom_traj,constraints,obj.ik_options);
      info 
      infeasible_constraint
    end

    function obj = generateConstraints(obj, q0, options)

      if nargin < 3
        plan_options = struct();
      end
      if ~isfield(options,'angle') options.angle = 0; end


      % need options for orientation, reach height etc. 
      R = rpy2rotmat([0;-options.angle;0]);
      quat_des = rotmat2quat(obj.T_wheel_2_world(1:3,1:3)*R);
      obj.hand_orientation_constraint = WorldQuatConstraint(obj.r,obj.r.findLinkId(obj.hand),quat_des,obj.options.quat_tol);
      obj.steering_position_constraint = cell(obj.N,1);

      % for j = 1:obj.N-1
      %   theta = options.angle + j/obj.N*2*pi;
      %   z = obj.options.turn_radius*cos(theta);
      %   x = obj.options.turn_radius*sin(theta);
      %   y = obj.options.reach_depth;
      %   lb = [x;y;z] - obj.options.tol*ones(3,1);
      %   ub = [x;y;z] + obj.options.tol*ones(3,1);
      %   obj.steering_position_constraint{j} = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,obj.T_wheel_2_world,lb,ub,[j,j+1/2]);
      % end

      joint_ind = setdiff([1:obj.nq]',obj.arm_idx);
      obj.posture_constraint = PostureConstraint(obj.r);
      obj.posture_constraint = obj.posture_constraint.setJointLimits(joint_ind,q0(joint_ind),q0(joint_ind));

      obj.final_posture_constraint = PostureConstraint(obj.r,[obj.N,obj.N+1/2]);
      obj.final_posture_constraint = obj.final_posture_constraint.setJointLimits([1:obj.nq]',q0,q0);
    end

    function hand_orientation_constraint = genHandQuatConstraintFromPose(obj,q0)
      kinsol = obj.r.doKinematics(q0);
      hand_pos = obj.r.forwardKin(kinsol,obj.r.findLinkId(obj.hand),obj.hand_pt,1);
      quat_des = rpy2quat(hand_pos(4:6));
      hand_orientation_constraint = WorldQuatConstraint(obj.r,obj.r.findLinkId(obj.hand),quat_des,obj.options.quat_tol);
    end

    function obj = setqdmax(obj,speed)
      if nargin < 2
        speed = 1;
      end

      max_degrees_per_second = 20;
      max_wrist_degrees_per_second = 60;
      obj.qd_max = max_degrees_per_second*pi/180*ones(obj.nq,1);
      obj.qd_max(obj.lwy_idx) = max_wrist_degrees_per_second*pi/180;
    end

    function [q0,x0] = getRobotState(obj)
      data = [];
      while isempty(data)
        data = obj.state_monitor.getNextMessage(10);
      end
      [x0,t] = obj.r.getStateFrame().lcmcoder.decode(data);
      q0 = x0(1:obj.nq);
    end


    function publishTraj(obj, qtraj,snopt_info)
      utime = now() * 24 * 60 * 60;
      nq_atlas = obj.r.getNumPositions;
      ts = qtraj.pp.breaks;
      q = qtraj.eval(ts);
      xtraj_atlas = zeros(2+2*nq_atlas,length(ts));
      xtraj_atlas(2+(1:nq_atlas),:) = q(1:nq_atlas,:);
      snopt_info_vector = snopt_info*ones(1, size(xtraj_atlas,2));
      ts = ts - ts(1);
      obj.plan_publisher.publish(xtraj_atlas, ts, utime, snopt_info_vector);
    end

  end

end