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
    arm_and_neck_inds = [];

    % Initializes on first getQPInput
    % (setup foot contact lock and upper body state to be
    % tracked)
    initialized = 0;

    DEBUG;
    SLOW_DRAW;

    last_ts = [];
    last_coefs = [];
    t_start = [];

    lc;

    foot_vertices = struct('right', [-0.05, 0.05, 0.05, -0.05; 
                                     -0.02, -0.02, 0.02, 0.02],...
                           'left', [-0.05, 0.05, 0.05, -0.05; 
                                     -0.02, -0.02, 0.02, 0.02]);
    reachable_vertices = struct('right', [-0.4, 0.4, 0.4, -0.4;
                                   -0.2, -0.2, -0.45, -0.45],...
                          'left', [-0.4, 0.4, 0.4, -0.4;
                                   0.2, 0.2, 0.45, 0.45]);

    % nonconstant as it will be reassigned if we're in sim mode
    CAPTURE_MAX_FLYFOOT_HEIGHT = 0.025;

  end

  properties (Constant)
    OTHER_FOOT = struct('right', 'left', 'left', 'right'); % make it easy to look up the other foot's name
    TERRAIN_CONTACT_THRESH = 0.01;
    POST_EXECUTION_DELAY = 0.1;
    CAPTURE_SHRINK_FACTOR = 0.9; % liberal to prevent foot-roll
    FOOT_HULL_COP_SHRINK_FACTOR = 0.6; % liberal to prevent foot-roll, should be same as the capture shrink factor?
    MAX_CONSIDERABLE_FOOT_SWING = 0.15; % strides with extrema farther than this are ignored
    U_MAX = 10;

    MIN_STEP_DURATION = 0.25;
  end

  methods
    function obj = QPReactiveRecoveryPlan(robot, options)
      checkDependency('iris');
      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options, struct('g', 9.81, 'debug', 0, 'slow_draw', 0, 'sim_mode', 1));
      if (options.sim_mode)
        disp('Initialized in sim mode!');
        obj.CAPTURE_MAX_FLYFOOT_HEIGHT = 0.0;
      end
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.robot = robot;
      obj.DEBUG = options.debug;
      obj.SLOW_DRAW = options.slow_draw;
      % obj.qtraj = qtraj;
      % obj.LIP_height = LIP_height;
      S = load(obj.robot.fixed_point_file);
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.default_qp_input.whole_body_data.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
      obj.arm_and_neck_inds = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck')];
      [~, obj.V, ~, obj.LIP_height] = obj.robot.planZMPController([0;0], obj.qtraj);
      obj.g = options.g;
      obj.point_mass_biped = PointMassBiped(sqrt(options.g / obj.LIP_height));
      obj.initialized = 0;
    end

    function obj = resetInitialization(obj)
      obj.initialized = 0;
      obj.last_plan = [];
      obj.last_qp_input = [];
      obj.last_ts = [];
      obj.last_coefs = [];
      obj.t_start = [];
    end

    function next_plan = getSuccessor(obj, t, x)
      next_plan = QPLocomotionPlan.from_standing_state(x, obj.robot);
    end

    function qp_input = getQPControllerInput(obj, t_global, x, rpc, contact_force_detected)

      q = x(1:rpc.nq);
      qd = x(rpc.nq + (1:rpc.nv));
      kinsol = doKinematics(obj.robot, q);

      [com, J] = obj.robot.getCOM(kinsol);
      comd = J * qd;

      t_plan = t_global - obj.t_start;

      r_ic = com(1:2) + comd(1:2) / obj.point_mass_biped.omega;


      % if obj.SLOW_DRAW
      %   obj.lcmgl.glColor3f(0.2,0.2,1.0);
      %   obj.lcmgl.sphere([com(1:2); 0], 0.01, 20, 20);

      %   obj.lcmgl.glColor3f(0.9,0.2,0.2);
      %   obj.lcmgl.sphere([r_ic; 0], 0.01, 20, 20);
      % end


      foot_states = struct('right', struct('xyz_quat', [], 'xyz_quatdot', [], 'contact', false),...
                          'left', struct('xyz_quat', [], 'xyz_quatdot', [], 'contact', false));
      for f = {'right', 'left'}
        foot = f{1};
        [pos, J] = obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.(foot), [0;0;0], 2);
        vel = J * qd;
        foot_states.(foot).xyz_quat = pos;
        foot_states.(foot).xyz_quatdot = vel;
        [foot_states.(foot).terrain_height, foot_states.(foot).terrain_normal] = obj.robot.getTerrainHeight(foot_states.(foot).xyz_quat(1:2));

        if contact_force_detected(obj.robot.foot_body_id.(foot))
          foot_states.(foot).contact = true;
        end
      end
      % force terrain heights to come from the foot that's in contact (if either)
      if (foot_states.right.contact)
        foot_states.right.terrain_height = foot_states.right.xyz_quat(3);
        foot_states.left.terrain_height = foot_states.right.xyz_quat(3);
      elseif (foot_states.left.contact)
        foot_states.right.terrain_height = foot_states.left.xyz_quat(3);
        foot_states.left.terrain_height = foot_states.left.xyz_quat(3);
      end
      % finally check foot contacts against the most reasonable terrain
      % heights we've been able to find
      % for f = {'right', 'left'}
      %   foot = f{1};
      %   [pos, ~] = obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.(foot), [0;0;0], 2);
      %   if pos(3) < foot_states.(foot).terrain_height + obj.TERRAIN_CONTACT_THRESH
      %     foot_states.(foot).contact = true;
      %   end
      % end
      
      % Initialize if we haven't, to get foot lock into a known state
      % and capture upper body pose to hold it through the plan
      if (~obj.initialized) 
        % Record current state of arm, neck to hold (roughly) throughout recovery
        obj.qtraj(obj.arm_and_neck_inds) = x(obj.arm_and_neck_inds);
        obj.initialized = 1;
      end

      is_captured = obj.icpError(r_ic, foot_states, obj.foot_vertices) < 1e-2

      if ~isempty(obj.last_plan) && t_plan < obj.last_ts(end)
        disp('continuing current plan')
        qp_input = obj.getInterceptInput(t_global, foot_states, rpc);
      elseif is_captured 
        disp('captured');
        qp_input = obj.getCaptureInput(t_global, r_ic, foot_states, rpc);
      elseif ~isempty(obj.last_plan) && t_plan < obj.last_ts(end) + obj.POST_EXECUTION_DELAY
        disp('in delay after plan end')
        qp_input = obj.getCaptureInput(t_global, r_ic, foot_states, rpc);
      else
        disp('replanning');
        intercept_plans = obj.getInterceptPlansmex(foot_states, r_ic, comd, obj.point_mass_biped.omega, obj.U_MAX);
        if isempty(intercept_plans)
          disp('recovery is not possible');
          qp_input = obj.getCaptureInput(t_global, r_ic, foot_states, rpc);
        else
          obj.last_plan = obj.chooseBestIntercept(intercept_plans);
          [obj.last_ts, obj.last_coefs] = obj.swingTraj(obj.last_plan, foot_states.(obj.last_plan.swing_foot));
          obj.t_start = t_global;
          qp_input = obj.getInterceptInput(t_global, foot_states, rpc);
        end
      end

      if (obj.DEBUG > 0)
        fprintf('starting publish for vis: ');
        t0 = tic();
        obj.publishForVisualization(t_global, com, r_ic, obj.last_ts, obj.last_coefs);
        toc(t0);
      end

      if obj.SLOW_DRAW
        obj.lcmgl.switchBuffers();
      end

      obj.last_qp_input = qp_input;

    end

    function draw_plan(obj, pp, foot_states, reachable_vertices, plan)
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
      obj.lcmgl.sphere([plan.r_cop; 0], 0.01, 20, 20);

      obj.lcmgl.glColor3f(0.9,0.2,0.2)
      obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES)
      obj.lcmgl.glVertex3f(plan.r_cop(1), plan.r_cop(2), 0);
      obj.lcmgl.glVertex3f(plan.r_ic_new(1), plan.r_ic_new(2), 0);
      obj.lcmgl.glEnd();

      obj.lcmgl.glColor3f(1.0,1.0,0.3);
      stance_foot = obj.OTHER_FOOT.(plan.swing_foot);
      R = quat2rotmat(foot_states.(stance_foot).xyz_quat(4:7));
      rpy = rotmat2rpy(R);
      reachable_vertices_in_world_frame = bsxfun(@plus, rotmat(rpy(3)) * reachable_vertices.(plan.swing_foot), foot_states.(stance_foot).xyz_quat(1:2));
      obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINE_LOOP)
      for j = 1:size(reachable_vertices_in_world_frame, 2)
        obj.lcmgl.glVertex3f(reachable_vertices_in_world_frame(1,j),...
                             reachable_vertices_in_world_frame(2,j),...
                             0);
      end
      obj.lcmgl.glEnd();
    end

    function qp_input = getInterceptInput(obj, t_global, foot_states, rpc)
      pp = mkpp(obj.last_ts, obj.last_coefs, 6);
      if obj.SLOW_DRAW
        obj.draw_plan(pp, foot_states, obj.reachable_vertices, obj.last_plan);
      end
      
      qp_input = obj.default_qp_input;
      qp_input.whole_body_data.q_des = obj.qtraj;
      qp_input.zmp_data.x0 = [mean([foot_states.right.xyz_quat(1:2), foot_states.left.xyz_quat(1:2)], 2);
                              0; 0];
      qp_input.zmp_data.y0 = obj.last_plan.r_cop;
      qp_input.zmp_data.S = obj.V.S;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);

      if strcmp(obj.last_plan.swing_foot, 'right')
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

      % Don't allow support if we are less than halfway through the plan
      t = t_global - obj.t_start;
      if t <= (obj.last_ts(end)/2)
        support_for_swing = obj.support_logic_maps.prevent_support;
      else
        support_for_swing = obj.support_logic_maps.only_if_force_sensed;
      end
      
      qp_input.support_data(end+1) = struct('body_id', obj.robot.foot_body_id.(obj.last_plan.swing_foot),...
                                     'contact_pts', [rpc.contact_groups{obj.robot.foot_body_id.(obj.last_plan.swing_foot)}.toe,...
                                                     rpc.contact_groups{obj.robot.foot_body_id.(obj.last_plan.swing_foot)}.heel],...
                                     'support_logic_map', support_for_swing,...
                                     'mu',obj.mu,...
                                     'contact_surfaces', 0);

      % swing foot
      qp_input.body_motion_data = struct('body_id', obj.robot.foot_frame_id.(obj.last_plan.swing_foot),...
                                         'ts', obj.t_start+obj.last_ts,...
                                         'coefs', obj.last_coefs,...
                                         'toe_off_allowed', false,...
                                         'in_floating_base_nullspace', true,...
                                         'control_pose_when_in_contact', false,...
                                         'quat_task_to_world', [1;0;0;0], ...
                                         'translation_task_to_world', [0;0;0], ...
                                         'xyz_kp_multiplier', [1;1;1], ...
                                         'xyz_damping_ratio_multiplier', [1;1;1], ...
                                         'expmap_kp_multiplier', 1, ...
                                         'expmap_damping_ratio_multiplier', 1, ...
                                         'weight_multiplier', [1;1;1;1;1;1]);

      pelvis_height = foot_states.(stance_foot).terrain_height + 0.84;

      foot_rpy = [quat2rpy(foot_states.right.xyz_quat(4:7)), quat2rpy(foot_states.left.xyz_quat(4:7))];
      pelvis_yaw = angleAverage(foot_rpy(3,1), foot_rpy(3,2));
      pelvis_xyz_exp = [0; 0; pelvis_height; quat2expmap(rpy2quat([0;0;pelvis_yaw]))];
      coefs_pelvis = cat(3, zeros(6,1,3), pelvis_xyz_exp);
      qp_input.body_motion_data(end+1) = struct('body_id', rpc.body_ids.pelvis,...
                                                'ts',  obj.t_start+obj.last_ts,...
                                                'coefs', repmat(coefs_pelvis, [1, length(obj.last_ts)-1, 1]),...
                                                'toe_off_allowed', false,...
                                                'in_floating_base_nullspace', false,...
                                                'control_pose_when_in_contact', false,...
                                                'quat_task_to_world', [1;0;0;0], ...
                                                'translation_task_to_world', [0;0;0], ...
                                                'xyz_kp_multiplier', [1;1;1], ...
                                                'xyz_damping_ratio_multiplier', [1;1;1], ...
                                                'expmap_kp_multiplier', 1, ...
                                                'expmap_damping_ratio_multiplier', 1, ...
                                                'weight_multiplier', [1;1;1;0;0;1]);
      qp_input.param_set_name = 'recovery';
    end

    function qp_input = getCaptureInput(obj, t_global, r_ic, foot_states, rpc)
      qp_input = obj.default_qp_input;
      qp_input.whole_body_data.q_des = obj.qtraj;
      qp_input.zmp_data.x0 = [mean([foot_states.right.xyz_quat(1:2), foot_states.left.xyz_quat(1:2)], 2);
                              0; 0];
      % qp_input.zmp_data.x0 = [0;0; 0; 0];
      qp_input.zmp_data.y0 = r_ic;
      qp_input.zmp_data.S = obj.V.S;
      qp_input.zmp_data.D = -obj.LIP_height/obj.g * eye(2);

      qp_input.support_data = struct('body_id', cell(1, 2),...
                                     'contact_pts', cell(1, 2),...
                                     'support_logic_map', cell(1, 2),...
                                     'mu', {obj.mu, obj.mu},...
                                     'contact_surfaces', {0, 0});
      qp_input.body_motion_data = struct('body_id', cell(1, 3),..._
                                         'ts', cell(1, 3),...
                                         'coefs', cell(1, 3),...
                                         'toe_off_allowed', cell(1,3),...
                                         'in_floating_base_nullspace', cell(1,3),...
                                         'control_pose_when_in_contact', cell(1,3));
      feet = {'right', 'left'};
      for j = 1:2
        foot = feet{j};
        sole_pose_quat = foot_states.(foot).xyz_quat;
        sole_xyz_exp = [sole_pose_quat(1:3); quat2expmap(sole_pose_quat(4:7))];
        sole_xyz_exp(3) = foot_states.(foot).terrain_height;

        qp_input.support_data(j).body_id = obj.robot.foot_body_id.(foot);
        qp_input.support_data(j).contact_pts = [rpc.contact_groups{obj.robot.foot_body_id.(foot)}.toe,...
                                                rpc.contact_groups{obj.robot.foot_body_id.(foot)}.heel];
        if foot_states.(foot).xyz_quat(3) - foot_states.(foot).terrain_height < obj.CAPTURE_MAX_FLYFOOT_HEIGHT
          qp_input.support_data(j).support_logic_map = obj.support_logic_maps.require_support;
        else
          qp_input.support_data(j).support_logic_map = obj.support_logic_maps.only_if_force_sensed;
        end

        qp_input.body_motion_data(j).body_id = obj.robot.foot_frame_id.(foot);
        qp_input.body_motion_data(j).ts = [t_global, t_global];
        qp_input.body_motion_data(j).coefs = cat(3, zeros(6,1,3), reshape(sole_xyz_exp, [6, 1, 1]));
        qp_input.body_motion_data(j).toe_off_allowed = false;
        qp_input.body_motion_data(j).in_floating_base_nullspace = true;
        qp_input.body_motion_data(j).control_pose_when_in_contact = false;
        qp_input.body_motion_data(j).quat_task_to_world =  [1;0;0;0];
        qp_input.body_motion_data(j).translation_task_to_world =  [0;0;0];
        qp_input.body_motion_data(j).xyz_kp_multiplier =  [1;1;1];
        qp_input.body_motion_data(j).xyz_damping_ratio_multiplier =  [1;1;1];
        qp_input.body_motion_data(j).expmap_kp_multiplier =  1;
        qp_input.body_motion_data(j).expmap_damping_ratio_multiplier =  1;
        qp_input.body_motion_data(j).weight_multiplier =  [1;1;1;1;1;1];
      end
      % warning('probably not right pelvis height if feet height differ...')
      pelvis_height = 0.5 * (foot_states.left.terrain_height + foot_states.right.terrain_height) + 0.84;

      foot_rpy = [quat2rpy(foot_states.right.xyz_quat(4:7)), quat2rpy(foot_states.left.xyz_quat(4:7))];
      pelvis_yaw = angleAverage(foot_rpy(3,1), foot_rpy(3,2));
      pelvis_xyz_exp = [0; 0; pelvis_height; quat2expmap(rpy2quat([0;0;pelvis_yaw]))];
      qp_input.body_motion_data(3) = struct('body_id', rpc.body_ids.pelvis,...
                                            'ts', t_global + [0, 0],...
                                            'coefs', cat(3, zeros(6,1,3), pelvis_xyz_exp),...
                                            'toe_off_allowed', false,...
                                            'in_floating_base_nullspace', false,...
                                            'control_pose_when_in_contact', false,...
                                            'quat_task_to_world', [1;0;0;0], ...
                                            'translation_task_to_world', [0;0;0], ...
                                            'xyz_kp_multiplier', [1;1;1], ...
                                            'xyz_damping_ratio_multiplier', [1;1;1], ...
                                            'expmap_kp_multiplier', 1, ...
                                            'expmap_damping_ratio_multiplier', 1, ...
                                            'weight_multiplier', [1;1;1;0;0;1]);
      qp_input.param_set_name = 'recovery';
    end

    function [ts, coefs] = swingTraj(obj, intercept_plan, foot_state)

      %if norm(intercept_plan.r_foot_new(1:2) - foot_state.pose(1:2)) < 0.05
      %  disp('Asking for swing traj of very short step')
        %if (foot_state.pose(3) < obj.robot.getTerrainHeight(foot_state.pose(1:2)) + obj.TERRAIN_CONTACT_THRESH)
          %disp('Foot seems to be in contact. Holding pose and planting foot')
          %intercept_plan.r_foot_new = foot_state.pose;
          %intercept_plan.r_foot_new(3) = obj.robot.getTerrainHeight(foot_state.pose(1:2));
        %end
      %end

      %fprintf('0,%f,%f\n', intercept_plan.tswitch, intercept_plan.tf);
      
      % generate swing traj, which requires figuring out knot points to feed in
      % which requries knowing roughly what the long-term plan is for a swing, so we can
      % figure out what phase we're in
      % phases:
      %   last: if we can just descend to our goal point, do it, with knot points
      %     along a quadratic arc down to the goal point. "Can just descend" means
      %     "A*z^2 >= (ground distance to goal)"
      %   second-to-last: if we can't just descend to our goal point, arc to it

      dist_to_goal = norm(intercept_plan.r_foot_new(1:2) - foot_state.xyz_quat(1:2));
      descend_coeff = (1/0.15)^2;

      if (descend_coeff*((foot_state.xyz_quat(3) - foot_state.terrain_height)^2) >= dist_to_goal)
        disp('case1');
        % descend straight there
        sizecheck(intercept_plan.r_foot_new, [7, 1]);
        fraction_first = 0.7;
        % TODO: need to subtract off terrain height
        swing_height_first = foot_state.xyz_quat(3)*(1-fraction_first^2);

        ts = [0 0 intercept_plan.tf];
        xs = zeros(6,3); % only plan one middle knot point
        xs(1:3,1) = foot_state.xyz_quat(1:3);
        xs(1:3,3) = intercept_plan.r_foot_new(1:3);
        [xs(4:6,1), dw0] = quat2expmap(foot_state.xyz_quat(4:7));
        xs(4:6,3) = quat2expmap(intercept_plan.r_foot_new(4:7));
        xd0 = [foot_state.xyz_quatdot(1:3); dw0 * foot_state.xyz_quatdot(4:7)];
        xdf = zeros(6,1);

        % TODO: should probably interpolate orientation
        xs(4:6, 2) = xs(4:6,1);
        xs(3, 2) = xs(3, 3) + swing_height_first;
        % interp position between first and last
        xs(1:2, 2) = (1-fraction_first)*xs(1:2, 1) + fraction_first*xs(1:2, 3);

        for j = 2:3
          xs(4:6,j) = unwrapExpmap(xs(4:6,j-1), xs(4:6,j));
        end

        [coefs, ts, ~] = nWaypointCubicSplineFreeKnotTimesmex(ts(1), ts(end), xs, xd0, xdf);

           
        if (obj.SLOW_DRAW)
          tt = linspace(ts(1), ts(end));
          pp = mkpp(ts, coefs, 6);
          ps = ppval(fnder(pp, 2), tt);
          fprintf('umax x:%f y: %f z:%f\n', max(ps(1, :)), max(ps(2, :)), max(ps(3, :)));

          obj.lcmgl.glColor3f(0.1,0.1,1.0);
          for k=1:3
            obj.lcmgl.sphere(xs(1:3, k).', 0.01, 20, 20);
          end
        end
      else
        disp('case2');
        swing_height_first = 0.03;
        swing_height_second = 0.03;
        fraction_first = 0.15;
        fraction_second = 0.85;
        if (foot_state.xyz_quat(3) > swing_height_first+foot_state.terrain_height)
          % interpolate between current foot height and second swing height
          swing_height_first = swing_height_second*(fraction_first/fraction_second) + ((foot_state.xyz_quat(3)-foot_state.terrain_height)*(1-fraction_first/fraction_second));
        end
        ts = [0 0 0 intercept_plan.tf];
        xs = zeros(6,4);
        xs(1:3,1) = foot_state.xyz_quat(1:3);
        xs(1:3,4) = intercept_plan.r_foot_new(1:3);
        [xs(4:6,1), dw0] = quat2expmap(foot_state.xyz_quat(4:7));
        xs(4:6,4) = quat2expmap(intercept_plan.r_foot_new(4:7));
        xd0 = [foot_state.xyz_quatdot(1:3); dw0 * foot_state.xyz_quatdot(4:7)];
        xdf = zeros(6,1);

        xs(4:6, 2) = xs(4:6,1);
        xs(4:6, 3) = xs(4:6,4);
        xs(3, 2) = foot_state.terrain_height + swing_height_first;
        xs(3, 3) = xs(3, 4) + swing_height_second;
        % interp position between first and last
        xs(1:2, 2) = (1-fraction_first)*xs(1:2, 1) + fraction_first*xs(1:2, 4);
        xs(1:2, 3) = (1-fraction_second)*xs(1:2, 1) + fraction_second*xs(1:2, 4);

        [coefs, ts, ~] = nWaypointCubicSplineFreeKnotTimesmex(ts(1), ts(end), xs, xd0, xdf);
        
        if (obj.SLOW_DRAW)
          obj.lcmgl.glColor3f(0.1,1.0,0.1);
          for k=1:4
            obj.lcmgl.sphere(xs(1:3, k).', 0.01, 20, 20);
          end

          tt = linspace(ts(1), ts(end));
          pp = mkpp(ts, coefs, 6);
          ps = ppval(fnder(pp, 2), tt);
          fprintf('umax x:%f y: %f z:%f\n', max(ps(1, :)), max(ps(2, :)), max(ps(3, :)));
        end


        if obj.DEBUG
          tt = linspace(ts(1), ts(end));
          pp = mkpp(ts, coefs, 6);
          ps = ppval(pp, tt);
          figure(10)
          clf
          subplot(311)
          plot(tt, ps(1,:), tt, ps(2,:), tt, ps(3,:));
          subplot(312)
          ps = ppval(fnder(pp, 1), tt);
          plot(tt, ps(1,:), tt, ps(2,:), tt, ps(3,:));
          subplot(313)
          ps = ppval(fnder(pp, 2), tt);
          plot(tt, ps(1,:), tt, ps(2,:), tt, ps(3,:));
        end
      end
      % pp = mkpp(ts, coefs, 6);

    end

    function publishForVisualization(obj, t, com, r_ic, ts, coefs)
      msg = drc.reactive_recovery_debug_t;
      msg.utime = t*1E9;
      msg.com = com;
      msg.icp = r_ic;
      msg.num_spline_ts = numel(ts);
      msg.num_spline_segments = msg.num_spline_ts - 1;
      msg.ts = ts;
      msg.coefs = coefs;
      obj.lc.publish('REACTIVE_RECOVERY_DEBUG', msg);
    end
  end

  methods(Static)
    function best_plan = chooseBestIntercept(intercept_plans)
      [~, idx] = min([intercept_plans.error]);
      best_plan = intercept_plans(idx);
    end
  end

  methods
    is_captured = isICPCaptured(obj, r_ic, foot_states, foot_vertices);
    intercept_plans = getInterceptPlansmex(obj, foot_states, foot_vertices, reachable_vertices, r_ic, comd, omega, u_max);
  end

  methods(Static)
    y = closestPointInConvexHull(x, V);
    xf = bangBangUpdate(x0, xd0, tf, u);
    x_ic_new = icpUpdate(x_ic, x_cop, dt, omega);
    [tf, tswitch, u] = bangBangIntercept(x0, xd0, xf, u_max);
    p = expTaylor(a, b, c, n);
    [t_int, l_int] = expIntercept(a, b, c, l0, ld0, u, n);

  end
end


