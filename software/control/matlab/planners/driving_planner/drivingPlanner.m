classdef drivingPlanner

  properties
    T_wheel
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
      obj.hand_pt = [0;0;0];
      obj.T_wheel = T;

      if obj.hand(1) == 'l'
        obj.arm_idx = obj.r.findPositionIndices('l_arm');
      else
        obj.arm_idx = obj.r.findPositionIndices('r_arm');
      end

      if ~isfield(options,'wheel_radius') obj.options.wheel_radius = 0.2; end
      if ~isfield(options,'turn_radius'), obj.options.turn_radius = obj.options.wheel_radius; end
      if ~isfield(options,'reach_depth'), obj.options.reach_depth = 0; end

      % need to allow to specify the turn radius

      obj.nq = r.getNumPositions;
      obj.non_arm_idx = setdiff([1:obj.nq],obj.arm_idx);

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
    end

    function qtraj = planReach(obj,q0,options)
      if nargin < 3
        options = struct();
      end
      % need to specify reach depth and orientation
      if isfield(options,'reach_depth'), obj.options.reach_depth = options.reach_depth; end
      if isfield(options,'R_hand'),
        obj.R_hand = options.R_hand;
      end
      obj.R_hand = rpy2rotmat([0;0;0]);
      obj.q0 = q0;

      if ~isfield(options,'xyz_des')
        options.xyz_des = [0;obj.options.reach_depth;obj.options.turn_radius];
      end

      % plan to a safe pregrasp
      % plan to reach with the desired orientation
      obj = obj.generateConstraints();
      position_constraint = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,options.xyz_des,options.xyz_des);
      constraints = {obj.posture_constraint,obj.hand_orientation_constraint, position_constraint};
      q_touch = obj.r.inverseKin(q_seed,q_seed,constraints{:},obj.ik_options);
      q_safe = obj.q_safe;
      q_safe(obj.non_arm_idx) = q0(obj.non_arm_idx);
      q_vals = [q0,q_safe,q_touch];
      qtraj = constructAndRescaleTrajectory(q_vals);
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

    function obj = generateConstraints(obj, q0)
      % need options for orientation, reach height etc. 
      quat_des = rotmat2quat(obj.R_hand*obj.T_wheel(1:3,1:3));
      obj.hand_orientation_constraint = WorldQuatConstraint(obj.r,obj.r.findLinkId(obj.hand),quat_des,obj.options.quat_tol);
      obj.steering_position_constraint = cell(N,1);

      for j = 1:N-1
        theta = theta_0 + j/N*2*pi;
        z = obj.options.turn_radius*cos(theta);
        x = obj.options.turn_radius*sin(theta);
        y = plan_options.reach_depth;
        lb = [x,y,z] - obj.options.tol*ones(3,1);
        ub = [x,y,z] + obj.options.tol*ones(3,1);
        obj.steering_position_constraint{j} = WorldPositionInFrameConstraint(obj.r,obj.r.findLinkId(obj.hand),obj.hand_pt,lb,ub,[j,j+1/2]);
      end

      joint_ind = setdiff([1:obj.nq],obj.arm_idx);
      obj.posture_constraint = PostureConstraint(obj.r);
      obj.posture_constraint = obj.posture_constraint.setJointLimits(joint_ind,q0(joint_ind),q0(joint_ind));

      obj.final_posture_constraint = PostureConstraint(obj.r);
      obj.final_posture_constraint = obj.final_posture_constraint.setJointLimits([1:obj.nq],q0,q0,[obj.N,obj.N+1/2]);
    end

    function obj = setqdmax(obj,speed)

    end

  end

end