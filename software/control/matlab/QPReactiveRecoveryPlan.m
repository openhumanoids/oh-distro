classdef QPReactiveRecoveryPlan < QPControllerPlan
  properties
    robot
    qtraj
    mu = 0.5;
    g = 9.81;
    V
    LIP_height;
    lcmgl = LCMGLClient('reactive_recovery')
    last_qp_input;
    last_plan;
  end

  properties (Constant)
    OTHER_FOOT = struct('right', 'left', 'left', 'right'); % make it easy to look up the other foot's name
  end

  methods
    function obj = QPReactiveRecoveryPlan(robot, ~, ~)
      obj.robot = robot;
      % obj.qtraj = qtraj;
      % obj.LIP_height = LIP_height;
      S = load(obj.robot.fixed_point_file);
      obj.qtraj = S.xstar(1:obj.robot.getNumPositions());
      obj.default_qp_input = atlasControllers.QPInputConstantHeight();
      obj.default_qp_input.whole_body_data.q_des = zeros(obj.robot.getNumPositions(), 1);
      obj.default_qp_input.whole_body_data.constrained_dofs = [findPositionIndices(obj.robot,'arm');findPositionIndices(obj.robot,'neck');findPositionIndices(obj.robot,'back_bkz');findPositionIndices(obj.robot,'back_bky')];
      [~, obj.V, ~, obj.LIP_height] = obj.robot.planZMPController([0;0], obj.qtraj);
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
      obj.lcmgl.glColor3f(0.2,0.2,1.0);
      obj.lcmgl.sphere([com(1:2); 0], 0.01, 20, 20);

      omega = sqrt(obj.g / obj.LIP_height);

      r_ic = com(1:2) + comd(1:2) / omega;
      obj.lcmgl.sphere([r_ic; 0], 0.01, 20, 20);


      foot_states = struct('right', struct('position', [], 'velocity', [], 'contact', false),...
                          'left', struct('position', [], 'velocity', [], 'contact', false));
      for f = {'right', 'left'}
        foot = f{1};
        [pos, J] = obj.robot.forwardKin(kinsol, obj.robot.foot_body_id.(foot), [0;0;0], 1);
        vel = J * qd;
        foot_states.(foot).position = pos;
        foot_states.(foot).velocity = vel;
        % warning('hard-coded foot height for contact')
        if pos(3) < 0.085
          foot_states.(foot).contact = true;
        end
      end

      % warning('hard-coded for atlas foot shape');
      foot_vertices = struct('right', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02],...
                             'left', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02]);
      reachable_vertices = struct('right', [-0.3, 0.35, 0.35, -0.3;
                                     -0.15, -0.15, -0.4, -0.4],...
                            'left', [-0.3, 0.35, 0.35, -0.3;
                                     0.15, 0.15, 0.4, 0.4]);
      warning('todo: compute plans for foot *sole*, not origin')
      intercept_plans = QPReactiveRecoveryPlan.getInterceptPlans(foot_states, foot_vertices, reachable_vertices, r_ic, omega, 10);

      % intercept_plans = QPReactiveRecoveryPlan.enumerateCaptureOptions(foot_contact, foot_vertices, foot_states, r_ic, 10, omega);
      best_plan = QPReactiveRecoveryPlan.chooseBestIntercept(intercept_plans);

      if isempty(best_plan)
        best_plan = obj.last_plan;
      else
        obj.last_plan = best_plan;
      end

      [ts, coefs] = QPReactiveRecoveryPlan.swingTraj(best_plan, foot_states.(best_plan.swing_foot));
      pp = mkpp(ts, coefs, 6);

      ts = linspace(pp.breaks(1), pp.breaks(end));
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

      obj.lcmgl.switchBuffers();

      qp_input = obj.default_qp_input;
      qp_input.whole_body_data.q_des = obj.qtraj;
      qp_input.zmp_data.x0 = [0;0; 0; 0];
      qp_input.zmp_data.y0 = best_plan.r_cop;
      qp_input.zmp_data.S = obj.V.S;

      if strcmp(best_plan.swing_foot, 'right')
        stance_foot = 'left';
      else
        stance_foot = 'right';
      end
      qp_input.support_data = struct('body_id', obj.robot.foot_body_id.(stance_foot),...
                                     'contact_pts', [rpc.contact_groups{obj.robot.foot_body_id.(stance_foot)}.toe,...
                                                     rpc.contact_groups{obj.robot.foot_body_id.(stance_foot)}.heel],...
                                     'support_logic_map', obj.support_logic_maps.require_support,...
                                     'mu', obj.mu,...
                                     'contact_surfaces', 0);
      qp_input.body_motion_data = struct('body_id', obj.robot.foot_body_id.(best_plan.swing_foot),...
                                         'ts', t_global + ts(1:2),...
                                         'coefs', coefs(:,1,:));

      % warning('no rotation');
      qp_input.body_motion_data(end+1) = struct('body_id', rpc.body_ids.pelvis,...
                                                'ts', t_global + ts(1:2),...
                                                'coefs', cat(3, zeros(6,1,3), [nan;nan;0.84;0;0;0]));
      qp_input.param_set_name = 'walking';

      obj.last_qp_input = qp_input;

    end
  end

  methods(Static)
    function y = closestPointInConvexHull(x, V)
      checkDependency('iris');
      if size(V, 1) > 3 || size(V, 2) > 8
        error('V is too large for our custom solver')
      end

      u = iris.least_distance.cvxgen_ldp(bsxfun(@minus, V, x));
      y = u + x;
    end

    function intercept_plans = getInterceptPlans(foot_states, foot_vertices, reach_vertices, r_ic, omega, u)
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
        new_plans = QPReactiveRecoveryPlan.getInterceptPlansForFoot(foot_states, swing_foot, foot_vertices, reach_vertices.(swing_foot), r_ic, omega, u);
        intercept_plans = [intercept_plans, new_plans];
      end
    end

    function intercept_plans = getInterceptPlansForFoot(foot_states, swing_foot, foot_vertices, reachable_vertices_in_stance_frame, r_ic, omega, u)
      stance_foot = QPReactiveRecoveryPlan.OTHER_FOOT.(swing_foot);

      % Find the center of pressure, which we'll place as close as possible to the ICP
      stance_foot_vertices_in_world = bsxfun(@plus,...
                                             rotmat(foot_states.(stance_foot).position(6)) * foot_vertices.(stance_foot),...
                                             foot_states.(stance_foot).position(1:2));
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
        foot_states_prime.(foot).position(1:2) = R * (foot_states.(foot).position(1:2) - r_cop);
        foot_states_prime.(foot).velocity(1:2) = R * foot_states.(foot).velocity(1:2);
        foot_vertices_prime.(foot) = R * foot_vertices_prime.(foot);
      end
      r_ic_prime = R * (r_ic - r_cop);
      assert(abs(r_ic_prime(2)) < 1e-6);

      reachable_vertices_in_world_frame = bsxfun(@plus, rotmat(foot_states.(stance_foot).position(6)) * reachable_vertices_in_stance_frame, foot_states.(stance_foot).position(1:2));
      reachable_vertices_prime = R * bsxfun(@minus, reachable_vertices_in_world_frame, r_cop);
      intercept_plans = QPReactiveRecoveryPlan.getLocalFrameIntercepts(foot_states_prime, swing_foot, foot_vertices_prime, reachable_vertices_prime, r_ic_prime, u, omega);
      
      Ri = inv(R);
      for j = 1:length(intercept_plans)
        intercept_plans(j).r_foot_new = [Ri * intercept_plans(j).r_foot_new + r_cop; foot_states.(swing_foot).position(3:6)];
        intercept_plans(j).r_ic_new = Ri * intercept_plans(j).r_ic_new + r_cop;
        intercept_plans(j).swing_foot = swing_foot;
        intercept_plans(j).r_cop = r_cop;
      end
    end

    function intercept_plans = getLocalFrameIntercepts(foot_states, swing_foot, foot_vertices, reachable_vertices, r_ic_prime, u_max, omega)
      OFFSET = 0.0;

      % r_ic(t) = (r_ic(0) - r_cop) e^(t*omega) + r_cop

      % figure(7)
      % clf

      r_cop_prime = [0;0];

      tt = linspace(0, 1);

      % subplot(212)
      xprime_axis_intercepts = QPReactiveRecoveryPlan.bangbang(foot_states.(swing_foot).position(2),...
                                                   foot_states.(swing_foot).velocity(2),...
                                                   0,...
                                                   u_max);
      min_time_to_xprime_axis = min([xprime_axis_intercepts.tf]);

      % subplot(211)
      % hold on
      % plot(tt, QPReactiveRecoveryPlan.icpUpdate(r_ic_prime(1), r_cop_prime(1), tt, omega) + OFFSET, 'r-')

      x0 = foot_states.(swing_foot).position(1);
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
      end

      % subplot(211)

      for u = [u_max, -u_max]
        [t_int, x_int] = QPReactiveRecoveryPlan.expIntercept((x_ic - x_cop), omega, x_cop + OFFSET, x0, xd0, u, 5);
        mask = false(size(t_int));
        for j = 1:numel(t_int)
          if isreal(t_int(j)) && t_int(j) >= min_time_to_xprime_axis && t_int(j) >= abs(xd0 / u)
            mask(j) = true;
          end
        end
        t_int = t_int(mask);
        x_int = x_int(mask);

        for j = 1:numel(x_int)
          r_foot_int = [x_int(j); 0];
          r_foot_reach = QPReactiveRecoveryPlan.closestPointInConvexHull(r_foot_int, reachable_vertices);
          % r_foot_reach = r_foot_int;

          intercepts = QPReactiveRecoveryPlan.bangbang(x0, xd0, r_foot_reach(1), u_max);
          if ~isempty(intercepts)
            [~, i] = min([intercepts.tswitch]); % if there are multiple options, take the one that switches sooner
            intercept = intercepts(i);

            intercept_plans(end+1) = struct('tf', intercept.tf,...
                                            'tswitch', intercept.tswitch,...
                                            'r_foot_new', r_foot_reach,...
                                            'r_ic_new', [QPReactiveRecoveryPlan.icpUpdate(x_ic, x_cop, intercept.tf, omega);
                                                         0]);
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

        tswitch = zeros(size(tf));
        for j = 1:numel(tf)
          tswitch(j) = 0.5 * (tf(j) - xd0 / u);
        end

        if numel(tf) > 1
          error('i don''t think there should ever be more than one feasible root');
        end
        if ~isempty(tf)
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
      if intercept_plan.tswitch > 0
        sizecheck(intercept_plan.r_foot_new, [6, 1]);
        swing_height = 0.05;
        slack = 10;
        % Plan a one- or two-piece polynomial spline to get to intercept_plan.r_foot_new with final velocity 0. 
        params = struct('r0', foot_state.position,...
                        'rd0',foot_state.velocity + [0;0;0.1;0;0;0],...
                        'rf',intercept_plan.r_foot_new,...
                        'rdf',[0;0;-0.1;0;0;0],...
                        'r_switch_lb', [foot_state.position(1:2) - slack;
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     foot_state.position(4:6)],...
                        'r_switch_ub', [foot_state.position(1:2) + slack;
                                     intercept_plan.r_foot_new(3) + swing_height + slack;
                                     foot_state.position(4:6)],...
                        'rd_switch_lb', -slack * ones(6,1),...
                        'rd_switch_ub', slack * ones(6,1),...
                        't_switch', intercept_plan.tswitch,...
                        't_f', intercept_plan.tf);
        settings = struct('verbose', 0);
        [vars, status] = freeSplineMex(params, settings);
        coefs = [cat(3, vars.C1_3, vars.C1_2, vars.C1_1, vars.C1_0), cat(3, vars.C2_3, vars.C2_2, vars.C2_1, vars.C2_0)];
        ts = [0, intercept_plan.tswitch, intercept_plan.tf];
      else
        ts = [0, intercept_plan.tf];
        coefs = cubicSplineCoefficients(intercept_plan.tf, foot_state.position, intercept_plan.r_foot_new, foot_state.velocity, zeros(6,1));
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

    function intercept_plans = enumerateCaptureOptions(foot_contact, foot_vertices, foot_state, r_ic, u, omega)
      reachable_l_minus_r = [-0.3, 0.35, 0.35, -0.3;
                             0.15, 0.15, 0.4, 0.4];
      checkDependency('iris');

      if foot_contact.right && foot_contact.left
        available_feet = struct('stance', {'right', 'left'},...
                                'swing', {'left', 'right'});
      elseif ~foot_contact.right
        available_feet = struct('stance', {'left'},...
                                'swing', {'right'});
      else
        available_feet = struct('stance', {'right'},...
                                'swing', {'left'});
      end
      
      % warning('hard-coding offset');
      offset = 0.15;

      intercept_plans = struct('foot', {}, 'r_cop', {}, 'u', {}, 'tf', {}, 'tswitch', {}, 'error', {}, 'r_foot_new', {}, 'r_ic_new', {});
      for j = 1:length(available_feet)
        stance_vertices = bsxfun(@plus,...
                                 foot_state.(available_feet(j).stance).position(1:2),...
                                 foot_vertices.(available_feet(j).stance));
        if size(stance_vertices, 2) > 4
          error('too long for custom solver');
        end
        r_cop = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                      stance_vertices, r_ic)) + r_ic;
        r_foot = foot_state.(available_feet(j).swing).position(1:2);
        rd_foot = foot_state.(available_feet(j).swing).velocity(1:2);

        reach_vertices = reachable_l_minus_r;
        if strcmp(available_feet(j).swing, 'right')
          reach_vertices(2,:) = -reach_vertices(2,:);
        end
        reach_vertices = bsxfun(@plus,...
                                foot_state.(available_feet(j).stance).position(1:2),...
                                reach_vertices);

        available_u = [-u, u];
        for k = 1:length(available_u)
          [t_int, tswitch, r_foot_new, r_ic_new] = QPReactiveRecoveryPlan.icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, available_u(k), offset);
          
          error('if t_int is empty, we should project the icp onto the reachable polygon to find a reasonable step to take')
          
          for i = 1:length(t_int)
            v_reach = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                            reach_vertices,...
                                                            r_foot_new(:,i)));
            if norm(v_reach) > 1e-3
              % Desired pose is not reachable, so find the closest point in reachable region
              r_foot_new(:,i) = v_reach + r_foot_new(:,i);
            end
            error('this needs additional logic to re-calculate the intercept times, r_ic_new, etc. for the new desired foot location. Otherwise, we might end up going to the edge of the reachable region, but only as t->inf');
          

            foot_vertices_new = bsxfun(@plus,...
                                    r_foot_new(:,i),...
                                    foot_vertices.(available_feet(j).stance));

            v_double_support = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                                     [stance_vertices, foot_vertices_new],...
                                                                     r_ic_new(:,i)));
            if norm(v_double_support) < 1e-3
              % We've brought the ICP into the support polygon
              r_cop_new = r_ic_new(:,i);
            else
              % We're going to have to take another step
              r_cop_new = iris.least_distance.cvxgen_ldp(bsxfun(@minus,...
                                                                foot_vertices_new,...
                                                                r_ic_new(:,i))) + r_ic_new(:,i);
            end

            % warning('hard-coded for Atlas and z=0 terrain')
            foot_position_new = [r_foot_new(:,i); 0.0811; 0;0;0];

            intercept_plans(end+1) = struct('foot', available_feet(j).swing,...
                                            'r_cop', r_cop,...
                                            'u', available_u(k),...
                                            'tf', t_int(i),...
                                            'tswitch', tswitch(i),...
                                            'error', norm(r_cop_new - r_ic_new(:,i)),...
                                            'r_foot_new', foot_position_new,...
                                            'r_ic_new', r_ic_new(:,i));
          end
        end
      end
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


      % figure(1)
      % clf
      % hold on
      % tt = linspace(0, max([t_int, 2]));
      % plot(tt, polyval(p, tt), 'g--');
      % plot(tt, a.*exp(b.*tt) + c, 'g-');
      % plot(tt, polyval(p_spline, tt), 'r-');
      % plot(t_int, polyval(p_spline, t_int), 'ro');
    end

    function [t_int, tswitch, r_foot_new, r_ic_new] = icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, u, offset)
      nhat = r_ic - r_cop;
      nhat = nhat / norm(nhat);

      l_ic = r_ic' * nhat;
      l_cop = r_cop' * nhat;
      l_foot = r_foot' * nhat;
      ld_foot = rd_foot' * nhat;

      % u_signed = [u, -u];
      % intercepts = struct('u', {u_signed}, 'ts', {{}, {}});
      % for j = 1:2
        % r_ic(t) = (r_ic(0) - r_cop) e^(t*omega) + r_cop
      [t_int, l_int] = QPReactiveRecoveryPlan.expIntercept(l_ic - l_cop, omega, l_cop + offset, l_foot, ld_foot, u, 5);
      % t_int = t_int(t_int >= abs(ld_foot / u));
      tswitch = 0.5 * (t_int - ld_foot / u);

      t_int
      r_foot_new = zeros(2, length(t_int));
      r_ic_new = zeros(2, length(t_int));
      for j = 1:length(t_int)
        r_foot_new(:,j) = l_int(j) * nhat + r_cop;
        r_ic_new(:,j) = (r_ic - r_cop) * exp(t_int(j) * omega) + r_cop;
      end

      % end
    end
  end
end


