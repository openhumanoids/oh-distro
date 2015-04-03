classdef QPReactiveRecoveryPlan < QPControllerPlan
  properties
    robot
    omega;
    qtraj;
    mu = 0.5;
    V
    g
    LIP_height;
    point_mass_biped;
    lcmgl = LCMGLClient('reactive_recovery')
    last_qp_input;
    last_plan;

    % Stateful foot contact lock
    % Used to determine how long a foot has been continuously in
    % contact (to the resolution of the rate at which this class
    % is asked for a qp input)
    l_foot_last_noncontact = 0;
    r_foot_last_noncontact = 0;
    l_foot_last_contact = 0;
    r_foot_last_contact = 0;
    l_foot_in_contact_lock = 0;
    r_foot_in_contact_lock = 0;

    % for debugging
    l_foot_contact_history = [];
    r_foot_contact_history = [];

  end

  properties (Constant)
    OTHER_FOOT = struct('right', 'left', 'left', 'right'); % make it easy to look up the other foot's name
    TERRAIN_CONTACT_THRESH = 0.003;
  end

  methods
    function obj = QPReactiveRecoveryPlan(robot, options)
      checkDependency('iris');
      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options, struct('g', 9.81));
      obj.robot = robot;
      % obj.qtraj = qtraj;
      % obj.LIP_height = LIP_height;
      S = load(obj.robot.fixed_point_file);
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.default_qp_input.whole_body_data.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
      [~, obj.V, ~, obj.LIP_height] = obj.robot.planZMPController([0;0], obj.qtraj);
      obj.g = options.g;
      obj.point_mass_biped = PointMassBiped(sqrt(options.g / obj.LIP_height));

      figure;
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = QPLocomotionPlan.from_standing_state(x, obj.robot);
    end

    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)

      DEBUG = true;
      HYST_MIN_CONTACT_TIME = 0.04;
      HYST_MIN_NONCONTACT_TIME = 0.04;

      q = x(1:rpc.nq);
      qd = x(rpc.nq + (1:rpc.nv));
      kinsol = doKinematics(obj.robot, q);

      [com, J] = obj.robot.getCOM(kinsol);
      comd = J * qd;


      r_ic = com(1:2) + comd(1:2) / obj.point_mass_biped.omega;


      if DEBUG
        obj.lcmgl.glColor3f(0.2,0.2,1.0);
        obj.lcmgl.sphere([com(1:2); 0], 0.01, 20, 20);

        obj.lcmgl.glColor3f(0.9,0.2,0.2);
        obj.lcmgl.sphere([r_ic; 0], 0.01, 20, 20);
      end


      foot_states = struct('right', struct('pose', [], 'velocity', [], 'contact', false),...
                          'left', struct('pose', [], 'velocity', [], 'contact', false));
      for f = {'right', 'left'}
        foot = f{1};
        [pos, J] = obj.robot.forwardKin(kinsol, obj.robot.foot_body_id.(foot), [0;0;0], 1);
        T_orig = poseRPY2tform(pos);
        T_frame = obj.robot.getFrame(obj.robot.foot_frame_id.(foot)).T;
        T_sole = T_orig * T_frame;
        pos = tform2poseRPY(T_sole);
        vel = J * qd;
        foot_states.(foot).pose = pos;
        foot_states.(foot).velocity = vel;
        if pos(3) < obj.robot.getTerrainHeight(foot_states.(foot).pose(1:2)) + obj.TERRAIN_CONTACT_THRESH
          foot_states.(foot).contact = true;
        end
        if contact_force_detected(obj.robot.foot_body_id.(foot))
          foot_states.(foot).contact = true;
        end
      end

      % Update and check against contact locks
      if (~foot_states.left.contact)
        obj.l_foot_last_noncontact = t_global;
      else
        obj.l_foot_last_contact = t_global;
      end
      if (~foot_states.right.contact)
        obj.r_foot_last_noncontact = t_global;
      else
        obj.r_foot_last_contact = t_global;
      end
      % State switching only when hysterisis thresholds are met
      % noncontact -> contact
      if (~obj.r_foot_in_contact_lock && t_global - obj.r_foot_last_noncontact > HYST_MIN_CONTACT_TIME)
        obj.r_foot_in_contact_lock = true;
      end
      if (~obj.l_foot_in_contact_lock && t_global - obj.l_foot_last_noncontact > HYST_MIN_CONTACT_TIME)
        obj.l_foot_in_contact_lock = true;
      end
      % contact -> noncontact
      if (obj.r_foot_in_contact_lock && t_global - obj.r_foot_last_contact > HYST_MIN_NONCONTACT_TIME)
        obj.r_foot_in_contact_lock = false;
      end
      if (obj.l_foot_in_contact_lock && t_global - obj.l_foot_last_contact > HYST_MIN_NONCONTACT_TIME)
        obj.l_foot_in_contact_lock = false;
      end
      fprintf('l: %d r: %d\n', obj.l_foot_in_contact_lock, obj.r_foot_in_contact_lock);

     % obj.l_foot_contact_history = [obj.l_foot_contact_history; 
     %     t_global - obj.l_foot_last_noncontact, t_global - obj.l_foot_last_contact, obj.l_foot_in_contact_lock, foot_states.left.contact];

     % obj.r_foot_contact_history = [obj.r_foot_contact_history; 
     %     t_global - obj.r_foot_last_noncontact, t_global - obj.r_foot_last_contact, obj.r_foot_in_contact_lock, foot_states.right.contact];

     % [obj.l_foot_contact_history obj.r_foot_contact_history]

      % commit contact from that filtering
      foot_states.right.contact = obj.r_foot_in_contact_lock;
      foot_states.left.contact = obj.l_foot_in_contact_lock;


      % warning('hard-coded for atlas foot shape');
      foot_vertices = struct('right', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02],...
                             'left', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02]);
      reachable_vertices = struct('right', [-0.4, 0.4, 0.4, -0.4;
                                     -0.15, -0.15, -0.4, -0.4],...
                            'left', [-0.4, 0.4, 0.4, -0.4;
                                     0.15, 0.15, 0.4, 0.4]);

      is_captured = obj.isICPCaptured(r_ic, foot_states, foot_vertices);

      if is_captured
        qp_input = obj.getCaptureInput(t_global, r_ic, foot_states, rpc);
      else
        
        % if the last plan is about to finish, just finish it first.
        if ~isempty(obj.last_plan) && obj.last_plan.tf > t_global && ...
            obj.last_plan.tf - t_global < 0.05
          best_plan = obj.last_plan;
        else
        
          U_MAX = 20;
          intercept_plans = obj.getInterceptPlans(foot_states, foot_vertices, reachable_vertices, r_ic, comd,  obj.point_mass_biped.omega, U_MAX);

          if isempty(intercept_plans)
            disp('recovery is not possible');
            qp_input = obj.last_qp_input;
            return;
          end

          % Add to the errors a new term reflecting distance of 
          % foot from down-projected com of robot
          for j=1:numel(intercept_plans)
            com_to_foot_error = norm(intercept_plans(j).r_foot_new(1:2) - com(1:2));
            intercept_plans(j).error = intercept_plans(j).error + 4*com_to_foot_error;
          end
          best_plan = QPReactiveRecoveryPlan.chooseBestIntercept(intercept_plans);

          if isempty(best_plan)
            best_plan = obj.last_plan;
          else
            obj.last_plan = best_plan;
          end
        end
        
        qp_input = obj.getInterceptInput(t_global, foot_states, reachable_vertices, best_plan, rpc);
      end

      obj.lcmgl.switchBuffers();

      obj.last_qp_input = qp_input;

    end

    function qp_input = getInterceptInput(obj, t_global, foot_states, reachable_vertices, best_plan, rpc)
      DEBUG = true;
      [ts, coefs] = QPReactiveRecoveryPlan.swingTraj(best_plan, foot_states.(best_plan.swing_foot));
      %ts
      % TODO: rather than applying a transform to coefs, just send body_motion_data for the sole point, not the origin
      %warning('this transform is incorrect if the foot is rotating');
%       for j = 1:size(coefs, 2)
%         T_sole_frame = obj.robot.getFrame(obj.robot.foot_frame_id.(best_plan.swing_foot)).T;
%         T_sole = poseRPY2tform(coefs(:,j,end));
%         T_origin = T_sole * inv(T_sole_frame);
%         coefs(:,j,end) = tform2poseRPY(T_origin);
%       end

      pp = mkpp(ts, coefs, 6);

      if DEBUG
        ts = linspace(pp.breaks(1), pp.breaks(end), 50);
        obj.lcmgl.glColor3f(1.0, 0.2, 0.2);
        obj.lcmgl.glLineWidth(1);
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
        ps = ppval(pp, ts);
        for j = 1:length(ts)-1
          obj.lcmgl.glVertex3f(ps(1,j), ps(2,j), ps(3,j));
          obj.lcmgl.glVertex3f(ps(1,j+1), ps(2,j+1), ps(3,j+1));
        end
        obj.lcmgl.glEnd();

        obj.lcmgl.glColor3f(0.2,1.0,0.2);
        obj.lcmgl.sphere([best_plan.r_cop; 0], 0.01, 20, 20);

        obj.lcmgl.glColor3f(0.9,0.2,0.2)
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES)
        obj.lcmgl.glVertex3f(best_plan.r_cop(1), best_plan.r_cop(2), 0);
        obj.lcmgl.glVertex3f(best_plan.r_ic_new(1), best_plan.r_ic_new(2), 0);
        obj.lcmgl.glEnd();

        obj.lcmgl.glColor3f(1.0,1.0,0.3);
        stance_foot = obj.OTHER_FOOT.(best_plan.swing_foot);
        reachable_vertices_in_world_frame = bsxfun(@plus, rotmat(foot_states.(stance_foot).pose(6)) * reachable_vertices.(best_plan.swing_foot), foot_states.(stance_foot).pose(1:2));
        obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINE_LOOP)
        for j = 1:size(reachable_vertices_in_world_frame, 2)
          obj.lcmgl.glVertex3f(reachable_vertices_in_world_frame(1,j),...
                               reachable_vertices_in_world_frame(2,j),...
                               0);
        end
        obj.lcmgl.glEnd();
        
      end
      
      qp_input = obj.default_qp_input;
      qp_input.whole_body_data.q_des = obj.qtraj;
      qp_input.zmp_data.x0 = [mean([foot_states.right.pose(1:2), foot_states.left.pose(1:2)], 2);
                              0; 0];
      % qp_input.zmp_data.x0 = [0;0; 0; 0];
      qp_input.zmp_data.y0 = best_plan.r_cop;
      qp_input.zmp_data.S = obj.V.S;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);

      if strcmp(best_plan.swing_foot, 'right')
        stance_foot = 'left';
      else
        stance_foot = 'right';
      end
      qp_input.support_data = struct('body_id', obj.robot.foot_body_id.(stance_foot),...
                                     'contact_pts', [rpc.contact_groups{obj.robot.foot_body_id.(stance_foot)}.toe,...
                                                     rpc.contact_groups{obj.robot.foot_body_id.(stance_foot)}.heel],...
                                     'support_logic_map', obj.support_logic_maps.require_support,...
                                     'mu',obj.mu,...
                                     'contact_surfaces', 0);
      qp_input.body_motion_data = struct('body_id', obj.robot.foot_frame_id.(best_plan.swing_foot),...
                                         'ts', t_global + ts(1:2),...
                                         'coefs', coefs(:,1,:));

      pelvis_height = obj.robot.getTerrainHeight(foot_states.(stance_foot).pose(1:2)) + 0.8;
      pelvis_yaw = angleAverage(foot_states.right.pose(6), foot_states.left.pose(6));
      qp_input.body_motion_data(end+1) = struct('body_id', rpc.body_ids.pelvis,...
                                                'ts', t_global + ts(1:2),...
                                                'coefs', cat(3, zeros(6,1,3), [nan;nan;pelvis_height;0;0;pelvis_yaw]));
      qp_input.param_set_name = 'recovery';
    end

    function qp_input = getCaptureInput(obj, t_global, r_ic, foot_states, rpc)
      qp_input = obj.default_qp_input;
      qp_input.whole_body_data.q_des = obj.qtraj;
      qp_input.zmp_data.x0 = [mean([foot_states.right.pose(1:2), foot_states.left.pose(1:2)], 2);
                              0; 0];
      % qp_input.zmp_data.x0 = [0;0; 0; 0];
      qp_input.zmp_data.y0 = r_ic;
      qp_input.zmp_data.S = obj.V.S;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);

      qp_input.support_data = struct('body_id', cell(1, 2),...
                                     'contact_pts', cell(1, 2),...
                                     'support_logic_map', cell(1, 2),...
                                     'mu', num2cell(obj.mu * ones(1, 2)),...
                                     'contact_surfaces', num2cell(zeros(1, 2)));
      qp_input.body_motion_data = struct('body_id', cell(1, 3),..._
                                         'ts', cell(1, 3),...
                                         'coefs', cell(1, 3));
      feet = {'right', 'left'};
      for j = 1:2
        foot = feet{j};
        qp_input.support_data(j).body_id = obj.robot.foot_body_id.(foot);
        qp_input.support_data(j).contact_pts = [rpc.contact_groups{obj.robot.foot_body_id.(foot)}.toe,...
                                                rpc.contact_groups{obj.robot.foot_body_id.(foot)}.heel];
        qp_input.support_data(j).support_logic_map = obj.support_logic_maps.kinematic_or_sensed;

        T_sole_frame = obj.robot.getFrame(obj.robot.foot_frame_id.(foot)).T;
        sole_pose = foot_states.(foot).pose;
        sole_pose(3) = obj.robot.getTerrainHeight(sole_pose(1:2));
        T_sole = poseRPY2tform(sole_pose);
        T_origin = T_sole * inv(T_sole_frame);
        origin_pose = tform2poseRPY(T_origin);

        qp_input.body_motion_data(j).body_id = obj.robot.foot_body_id.(foot);
        qp_input.body_motion_data(j).ts = [t_global, t_global];
        qp_input.body_motion_data(j).coefs = cat(3, zeros(6,1,3), reshape(origin_pose, [6, 1, 1]));
      end
      % warning('probably not right pelvis height if feet height differ...')
      pelvis_height = (obj.robot.getTerrainHeight(foot_states.left.pose(1:2)) + obj.robot.getTerrainHeight(foot_states.right.pose(1:2)))/2 + 0.84;
      pelvis_yaw = angleAverage(foot_states.right.pose(6), foot_states.left.pose(6));
      qp_input.body_motion_data(3) = struct('body_id', rpc.body_ids.pelvis,...
                                            'ts', t_global + [0, 0],...
                                            'coefs', cat(3, zeros(6,1,3), [nan;nan;pelvis_height;0;0;pelvis_yaw]));
      qp_input.param_set_name = 'recovery';
    end

    function is_captured = isICPCaptured(obj, r_ic, foot_states, foot_vertices)
      SHRINK_FACTOR = 0.9;

      all_vertices_in_world = zeros(2,0);

      for f = {'right', 'left'}
        foot = f{1};
        if foot_states.(foot).pose(3) < ...
            obj.robot.getTerrainHeight(foot_states.(foot).pose(1:2)) + obj.TERRAIN_CONTACT_THRESH
          R = rotmat(foot_states.(foot).pose(6));
          foot_vertices_in_world = bsxfun(@plus,...
                                          SHRINK_FACTOR * R * foot_vertices.(foot),...
                                          foot_states.(foot).pose(1:2));
          all_vertices_in_world = [all_vertices_in_world, foot_vertices_in_world];
        end
      end

      if (isempty(all_vertices_in_world))
        is_captured = 0;
      else
        u = iris.least_distance.cvxgen_ldp(bsxfun(@minus, all_vertices_in_world, r_ic));
        is_captured = norm(u) < 1e-2;
      end
    end

    function intercept_plans = getInterceptPlans(obj, foot_states, foot_vertices, reach_vertices, r_ic, comd, omega, u)
      intercept_plans = struct('tf', {},...
                               'tswitch', {},...
                               'r_foot_new', {},...
                               'r_ic_new', {},...
                               'error', {},...
                               'swing_foot', {},...
                               'r_cop', {});
      if foot_states.right.contact && foot_states.left.contact
        available_feet = struct('stance', {'right', 'left'},...
                                'swing', {'left', 'right'});
      elseif ~foot_states.right.contact
        available_feet = struct('stance', {'left'},...
                                'swing', {'right'});
      else
        available_feet = struct('stance', {'right'},...
                                'swing', {'left'});
      end

      for j = 1:length(available_feet)
        swing_foot = available_feet(j).swing;
        new_plans = obj.getInterceptPlansForFoot(foot_states, swing_foot, foot_vertices, reach_vertices.(swing_foot), r_ic, comd, omega, u);
        if ~isempty(new_plans)
          intercept_plans = [intercept_plans, new_plans];
        end
      end
    end

    function intercept_plans = getInterceptPlansForFoot(obj, foot_states, swing_foot, foot_vertices, reachable_vertices_in_stance_frame, r_ic, comd, omega, u)
      stance_foot = QPReactiveRecoveryPlan.OTHER_FOOT.(swing_foot);

      % Find the center of pressure, which we'll place as close as possible to the ICP
      stance_foot_vertices_in_world = bsxfun(@plus,...
                                             rotmat(foot_states.(stance_foot).pose(6)) * foot_vertices.(stance_foot),...
                                             foot_states.(stance_foot).pose(1:2));
      r_cop = QPReactiveRecoveryPlan.closestPointInConvexHull(r_ic, stance_foot_vertices_in_world);
      % r_ic - r_cop

      % Now transform the problem so that the x axis is aligned with (r_ic - r_cop)
      xprime = (r_ic - r_cop) / norm(r_ic - r_cop);
      yprime = [0, -1; 1, 0] * xprime;
      R = [xprime'; yprime'];
      foot_states_prime = foot_states;
      foot_vertices_prime = foot_vertices;
      for f = fieldnames(foot_states)'
        foot = f{1};
        foot_states_prime.(foot).pose(1:2) = R * (foot_states.(foot).pose(1:2) - r_cop);
        foot_states_prime.(foot).velocity(1:2) = R * foot_states.(foot).velocity(1:2);
        foot_vertices_prime.(foot) = R * foot_vertices_prime.(foot);
      end
      r_ic_prime = R * (r_ic - r_cop);
      assert(abs(r_ic_prime(2)) < 1e-6);

      reachable_vertices_in_world_frame = bsxfun(@plus, rotmat(foot_states.(stance_foot).pose(6)) * reachable_vertices_in_stance_frame, foot_states.(stance_foot).pose(1:2));
      reachable_vertices_prime = R * bsxfun(@minus, reachable_vertices_in_world_frame, r_cop);
      intercept_plans = QPReactiveRecoveryPlan.getLocalFrameIntercepts(foot_states_prime, swing_foot, foot_vertices_prime, reachable_vertices_prime, r_ic_prime, u, omega);

      Ri = inv(R);
      foot_avg_direction = angleAverage(foot_states.left.pose(6), foot_states.right.pose(6));
      % Desired foot direction is along direction of icp, or the opposite
      % (so we either stumble forward along it or backward, not sideways).
      if (norm(comd) > 0.25)
        comd = comd / norm(comd);
        desired_foot_direction = atan2(comd(2), comd(1));
        if (abs(angleDiff(desired_foot_direction, foot_avg_direction)) > pi/2)
          desired_foot_direction = atan2(-comd(2), -comd(1));
        end
      else
        desired_foot_direction = foot_avg_direction;
      end
      
      for j = 1:length(intercept_plans)
        % rotate foot to be pointing STEP degrees closer to desired foot
        % dir
        new_foot_dir_vec = [cos(foot_states.(stance_foot).pose(6)); sin(foot_states.(stance_foot).pose(6))];
        err = angleDiff(foot_states.(stance_foot).pose(6), desired_foot_direction);
        err = sign(err)*min(abs(err), pi/8);
        new_foot_dir_vec = rotmat(err)*new_foot_dir_vec;
        new_foot_yaw = atan2(new_foot_dir_vec(2), new_foot_dir_vec(1));
        foot_states.(stance_foot).pose(6)
        intercept_plans(j).r_foot_new = [Ri * intercept_plans(j).r_foot_new + r_cop; 
                                         0;
                                         foot_states.(stance_foot).pose(4:5);
                                         new_foot_yaw];
        intercept_plans(j).r_foot_new(3) = obj.robot.getTerrainHeight(intercept_plans(j).r_foot_new(1:2));
        intercept_plans(j).r_ic_new = Ri * intercept_plans(j).r_ic_new + r_cop;
        intercept_plans(j).swing_foot = swing_foot;
        intercept_plans(j).r_cop = r_cop;
      end
      
    end

  end

  methods(Static)
    function y = closestPointInConvexHull(x, V)
      if size(V, 1) > 3 || size(V, 2) > 8
        error('V is too large for our custom solver')
      end

      u = iris.least_distance.cvxgen_ldp(bsxfun(@minus, V, x));
      y = u + x;
    end

    function intercept_plans = getLocalFrameIntercepts(foot_states, swing_foot, foot_vertices, reachable_vertices, r_ic_prime, u_max, omega)
      OFFSET = 0.1;

      % r_ic(t) = (r_ic(0) - r_cop) e^(t*omega) + r_cop

      % figure(7)
      % clf

      r_cop_prime = [0;0];


      % subplot(212)
      xprime_axis_intercepts = QPReactiveRecoveryPlan.bangbang(foot_states.(swing_foot).pose(2),...
                                                   foot_states.(swing_foot).velocity(2),...
                                                   0,...
                                                   u_max);
      min_time_to_xprime_axis = min([xprime_axis_intercepts.tf]);

      % subplot(211)
      % hold on
%       tt = linspace(0, 1);
      % plot(tt, QPReactiveRecoveryPlan.icpUpdate(r_ic_prime(1), r_cop_prime(1), tt, omega) + OFFSET, 'r-')

      x0 = foot_states.(swing_foot).pose(1);
      xd0 = foot_states.(swing_foot).velocity(1);

      intercept_plans = struct('tf', {}, 'tswitch', {}, 'r_foot_new', {}, 'r_ic_new', {});

      t_int = min_time_to_xprime_axis;
      x_ic = r_ic_prime(1);
      x_cop = r_cop_prime(1);
      x_ic_int = QPReactiveRecoveryPlan.icpUpdate(x_ic, x_cop, t_int, omega) + OFFSET;
      x_foot_int = [QPReactiveRecoveryPlan.bangbangXf(x0, xd0, t_int, u_max),...
                  QPReactiveRecoveryPlan.bangbangXf(x0, xd0, t_int, -u_max)];

      if x_ic_int >= min(x_foot_int) && x_ic_int <= max(x_foot_int)
        % The time to get onto the xprime axis dominates, so we can hit the ICP as soon as we get to that axis

        intercepts = QPReactiveRecoveryPlan.bangbang(x0, xd0, x_ic_int, u_max);

        if ~isempty(intercepts)
          [~, i] = min([intercepts.tswitch]); % if there are multiple options, take the one that switches sooner
          intercept = intercepts(i);

          r_foot_int = [x_ic_int; 0];
          r_foot_reach = QPReactiveRecoveryPlan.closestPointInConvexHull(r_foot_int, reachable_vertices);
          % r_foot_reach = r_foot_int;


          intercept_plans(end+1) = struct('tf', t_int,...
                                          'tswitch', intercept.tswitch,...
                                          'r_foot_new', r_foot_reach,...
                                          'r_ic_new', [x_ic_int; 0]);
        end
      else
        for u = [u_max, -u_max]
          [t_int, x_int] = QPReactiveRecoveryPlan.expIntercept((x_ic - x_cop), omega, x_cop + OFFSET, x0, xd0, u, 7);
          mask = false(size(t_int));
          for j = 1:numel(t_int)
            if isreal(t_int(j)) && t_int(j) >= min_time_to_xprime_axis && t_int(j) >= abs(xd0 / u)
              mask(j) = true;
            end
          end
          t_int = t_int(mask);
          x_int = x_int(mask);
          
          % Pre-generate r_foot_reaches
          
          % If there are no intercepts, go as far along the cop->ic line
          % out of our reachable set as possible.
          if isempty(t_int)
            r_foot_reaches = QPReactiveRecoveryPlan.closestPointInConvexHull([x_ic + OFFSET; 0], reachable_vertices);
            x_int = r_foot_reaches(1);
          else
            r_foot_reaches = zeros( 2, min(1, numel(x_int)) );
            for j = 1:numel(x_int)
              r_foot_int = [x_int(j); 0];
              y = iris.least_distance.cvxgen_ldp(bsxfun(@minus, reachable_vertices, r_foot_int));
              if norm(y) < 1e-3
                r_foot_reaches(:, j) = r_foot_int;
              else
                % TODO: is this the right thing to do?
                r_foot_reaches(:, j) = QPReactiveRecoveryPlan.closestPointInConvexHull([x_ic + OFFSET; 0], reachable_vertices);
              end
            end
          end
         
          for j = 1:numel(x_int)
            r_foot_reach = r_foot_reaches(:, j);
            r_foot_int = [x_int(j); 0];
            y = iris.least_distance.cvxgen_ldp(bsxfun(@minus, reachable_vertices, r_foot_int));
            if norm(y) < 1e-3
              r_foot_reach = r_foot_int;
            else
              % TODO: is this the right thing to do?
              r_foot_reach = QPReactiveRecoveryPlan.closestPointInConvexHull([x_ic + OFFSET; 0], reachable_vertices);
            end
            % r_foot_reach = QPReactiveRecoveryPlan.closestPointInConvexHull(r_foot_int, reachable_vertices);
            % r_foot_reach = r_foot_int;

            intercepts = QPReactiveRecoveryPlan.bangbang(x0, xd0, r_foot_reach(1), u_max);
            if ~isempty(intercepts)
              [~, i] = min([intercepts.tswitch]); % if there are multiple options, take the one that switches sooner
              intercept = intercepts(i);

              if ~valuecheck(x0 + 0.5 * xd0 * intercept.tf + 0.25 * intercept.u * intercept.tf^2 - 0.25 * xd0^2 / intercept.u, r_foot_reach(1))
                keyboard()
              end

              intercept.tf = max([intercept.tf, min_time_to_xprime_axis]);
              intercept_plans(end+1) = struct('tf', intercept.tf,...
                                              'tswitch', intercept.tswitch,...
                                              'r_foot_new', r_foot_reach,...
                                              'r_ic_new', [QPReactiveRecoveryPlan.icpUpdate(x_ic, x_cop, intercept.tf, omega);
                                                           0]);
            end
          end
        end
      end


      % for u = [u_max, -u_max]
      %   tt = linspace(abs(xd0 / u), abs(xd0 / u) + 1);
      %   plot(tt, QPReactiveRecoveryPlan.bangbangXf(x0, xd0, tt, u), 'g-');

      % end

      % plot([min_time_to_xprime_axis,...
      %       min_time_to_xprime_axis], ...
      %      [QPReactiveRecoveryPlan.bangbangXf(x0, xd0, min_time_to_xprime_axis, u_max),...
      %       QPReactiveRecoveryPlan.bangbangXf(x0, xd0, min_time_to_xprime_axis, -u_max)], 'r-')

      % for j = 1:length(intercept_plans)
      %   plot(intercept_plans(j).tf, intercept_plans(j).r_foot_new(1), 'ro');
      %   plot(intercept_plans(j).tf, intercept_plans(j).r_ic_new(1), 'r*');
      % end

      for j = 1:length(intercept_plans)
        intercept_plans(j).error = norm(intercept_plans(j).r_foot_new - (intercept_plans(j).r_ic_new + [OFFSET; 0]));
      end

    end

    function xf = bangbangXf(x0, xd0, tf, u)
      xf = x0 + 0.5 * xd0 .* tf + 0.25 * u * tf.^2 - 0.25 * xd0.^2 / u;
    end

    function x_ic_new = icpUpdate(x_ic, x_cop, dt, omega)
      x_ic_new = (x_ic - x_cop) * exp(dt*omega) + x_cop;
    end

    function intercepts = bangbang(x0, xd0, xf, u_max)
      % xf = x0 + 1/2 xd0 tf + 1/4 u tf^2 - 1/4 xd0^2 / u
      % 1/4 u tf^2 + 1/2 xd0 tf + x0 - xf - 1/4 xd0^2 / u = 0
      % a = 1/4 u
      % b = 1/2 xd0
      % c = x0 - xf - 1/4 xd0^2 / u
      intercepts = struct('tf', {}, 'tswitch', {}, 'u', {});

      % hold on

      for u = [u_max, -u_max]
        a = 0.25 * u;
        b = 0.5 * xd0;
        c = x0 - xf - 0.25 * xd0^2 / u;


        t_roots = [(-b + sqrt(b^2 - 4*a*c)) / (2*a), (-b - sqrt(b^2 - 4*a*c)) / (2*a)];
        mask = false(size(t_roots));
        for j = 1:numel(t_roots)
          if isreal(t_roots(j)) && t_roots(j) >= abs(xd0 / u)
            mask(j) = true;
          end
        end
        tf = unique(t_roots(mask));
        if numel(tf) == 2 && abs(tf(1) - tf(2)) < 1e-3
          tf = tf(1);
        end

        % tf(tf < abs(xd0 / u)) = abs(xd0 / u)

        if numel(tf) > 1
          error('i don''t think there should ever be more than one feasible root');
        end

        if ~isempty(tf)
%           valuecheck(x0 + 1/2 * xd0 * tf + 1/4 * u * tf^2 - 1/4 * xd0^2 / u, xf, 1e-6);
          tswitch = 0.5 * (tf - xd0 / u);
          intercepts(end+1) = struct('tf', tf, 'tswitch', tswitch, 'u', u);
        end

        % tt = linspace(abs(xd0 / u), max([abs(xd0/u) + 0.3, tf]));

        % plot(tt, a*tt.^2 + b*tt + c + xf, 'g-')

        % roots([a; b; c])
        % xswitch = x0 + xd0 * tswitch + 0.5 * u * tswitch.^2;
        % xdswitch = xd0 + u * tswitch;

        % for j = 1:numel(tswitch)
        %   tt = linspace(0, tswitch(j));
        %   plot(tt, x0 + xd0 * tt + 0.5 * u * tt.^2);

        %   tt = linspace(tswitch(j), tf(j));
        %   plot(tt, xswitch + xdswitch * (tt - tswitch(j)) + 0.5 * -1 * u * (tt - tswitch(j)).^2);

        %   plot(tf(j), xf, 'ro');
        % end
      end
    end

    function [ts, coefs] = swingTraj(intercept_plan, foot_state)
      DEBUG = false;

      if norm(intercept_plan.r_foot_new(1:2) - foot_state.pose(1:2)) < 0.05
        disp('here');
      end
      if intercept_plan.tswitch > 0.05 && (intercept_plan.tf - intercept_plan.tswitch) > 0.05
        sizecheck(intercept_plan.r_foot_new, [6, 1]);
        swing_height = 0.05;
        slack = 10;
        % Plan a one- or two-piece polynomial spline to get to intercept_plan.r_foot_new with final velocity 0.
        switch_ratio = intercept_plan.tswitch / intercept_plan.tf
        switch_angle = angleWeightedAverage(foot_state.pose(4:6), switch_ratio, intercept_plan.r_foot_new(4:6), 1-switch_ratio);
        params = struct('r0', foot_state.pose,...
                        'rd0',foot_state.velocity + [0;0;0;0;0;0],...
                        'rf',intercept_plan.r_foot_new,...
                        'rdf',[0;0;0;0;0;0],...
                        'r_switch_lb', [foot_state.pose(1:2) - slack;
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     switch_angle],...
                        'r_switch_ub', [foot_state.pose(1:2) + slack;
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     switch_angle],...
                        'rd_switch_lb', [-slack * ones(2,1); zeros(4,1)],...
                        'rd_switch_ub', [slack * ones(2,1); zeros(4,1)],...
                        't_switch', intercept_plan.tswitch,...
                        't_f', intercept_plan.tf);
        settings = struct('verbose', 0);
        [vars, status] = freeSplineMex(params, settings);
        coefs = [cat(3, vars.C1_3, vars.C1_2, vars.C1_1, vars.C1_0), cat(3, vars.C2_3, vars.C2_2, vars.C2_1, vars.C2_0)];
        ts = [0, intercept_plan.tswitch, intercept_plan.tf];
        
        if DEBUG
          tt = linspace(ts(1), ts(end));
          pp = mkpp(ts, coefs, 6);
          ps = ppval(pp, tt);
          figure(10)
          clf
          subplot(311)
          plot(tt, ps(1,:), tt, ps(2,:));
          subplot(312)
          ps = ppval(fnder(pp, 1), tt);
          plot(tt, ps(1,:), tt, ps(2,:));
          subplot(313)
          ps = ppval(fnder(pp, 2), tt);
          plot(tt, ps(1,:), tt, ps(2,:));
        end
        
      elseif norm(intercept_plan.r_foot_new(1:2) - foot_state.pose(1:2)) > 0.05
        tswitch = intercept_plan.tf / 2;
        swing_height = min([0.05, 0.25 * norm(intercept_plan.r_foot_new(1:2) - foot_state.pose(1:2))]);
        slack = 10;
        % Plan a one- or two-piece polynomial spline to get to intercept_plan.r_foot_new with final velocity 0. 
        params = struct('r0', foot_state.pose,...
                        'rd0',foot_state.velocity + [0;0;0;0;0;0],...
                        'rf',intercept_plan.r_foot_new,...
                        'rdf',[0;0;0;0;0;0],...
                        'r_switch_lb', [foot_state.pose(1:2) - slack;
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     intercept_plan.r_foot_new(4:6)],...
                        'r_switch_ub', [foot_state.pose(1:2) + slack;
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     intercept_plan.r_foot_new(4:6)],...
                        'rd_switch_lb', [-slack * ones(3,1); zeros(3,1)],...
                        'rd_switch_ub', [slack * ones(3,1); zeros(3,1)],...
                        't_switch', tswitch,...
                        't_f', intercept_plan.tf);
        settings = struct('verbose', 0);
        [vars, status] = freeSplineMex(params, settings);
        coefs = [cat(3, vars.C1_3, vars.C1_2, vars.C1_1, vars.C1_0), cat(3, vars.C2_3, vars.C2_2, vars.C2_1, vars.C2_0)];
        ts = [0, tswitch, intercept_plan.tf];
      else
        ts = [0, intercept_plan.tf];
        coefs = cubicSplineCoefficients(intercept_plan.tf, foot_state.pose, intercept_plan.r_foot_new, foot_state.velocity, zeros(6,1));
      end

      % pp = mkpp(ts, coefs, 6);

      % tt = linspace(0, intercept_plan.tf);
      % ps = ppval(pp, tt);
      % p_knot = ppval(pp, ts);
      % figure(4)
      % clf
      % hold on
      % for j = 1:6
      %   subplot(6, 1, j)
      %   hold on
      %   plot(tt, ps(j,:));
      %   plot(ts, p_knot(j,:), 'ro');
      % end
    end

    function best_plan = chooseBestIntercept(intercept_plans)
      [min_error, idx] = min([intercept_plans.error]);
      best_plan = intercept_plans(idx);
    end

    function p = expTaylor(a, b, c, n)
      % Taylor expansion of a*exp(b*x) + c about x=0 up to degree n
      p = zeros(n+1, length(a));
      for j = 1:n+1
        d = (n+1) - j;
        p(j,:) = a.*b.^d;
        if d == 0
          p(j,:) = p(j,:) + c;
        end
        p(j,:) = p(j,:) / factorial(d);
      end
    end

    function [t_int, l_int] = expIntercept(a, b, c, l0, ld0, u, n)
      DEBUG = false;

      % Find the t >= 0 solutions to a*e^(b*t) + c == l0 + 1/2*ld0*t + 1/4*u*t^2 - 1/4*ld0^2/u
      % using a taylor expansion up to power n
      p = QPReactiveRecoveryPlan.expTaylor(a, b, c, n);
      p_spline = [zeros(n-2, 1);
                  0.25 * u;
                  0.5 * ld0;
                  l0 - 0.25 * ld0.^2 / u];
      t_int = roots(p - p_spline);
      mask = false(size(t_int));
      for j = 1:size(t_int)
        mask(j) = isreal(t_int(j)) && t_int(j) > 0;
      end
      t_int = t_int(mask)';
      l_int = polyval(p_spline, t_int);

      if DEBUG
        figure(1)
        clf
        hold on
        tt = linspace(0, max([t_int, 0.5]));
        plot(tt, polyval(p, tt), 'g--');
        plot(tt, a.*exp(b.*tt) + c, 'g-');
        plot(tt, polyval(p_spline, tt), 'r-');
        plot(t_int, polyval(p_spline, t_int), 'ro');
      end
    end
  end
end


