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


      foot_state = struct('right', struct('position', [], 'velocity', []),...
                          'left', struct('position', [], 'velocity', []));
      foot_contact = struct('right', false,...
                            'left', false);
      for f = {'right', 'left'}
        foot = f{1};
        [pos, J] = obj.robot.forwardKin(kinsol, obj.robot.foot_body_id.(foot), [0;0;0], 1);
        vel = J * qd;
        foot_state.(foot).position = pos;
        foot_state.(foot).velocity = vel;
        % warning('hard-coded foot height for contact')
        if pos(3) < 0.085
          foot_contact.(foot) = true;
        end
      end

      % warning('hard-coded for atlas foot shape');
      foot_vertices = struct('right', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02],...
                             'left', [-0.05, 0.05, 0.05, -0.05; 
                                       -0.02, -0.02, 0.02, 0.02]);
      intercept_plans = QPReactiveRecoveryPlan.enumerateCaptureOptions(foot_contact, foot_vertices, foot_state, r_ic, 10, omega);
      best_plan = QPReactiveRecoveryPlan.chooseBestIntercept(intercept_plans);

      if isempty(best_plan)
        best_plan = obj.last_plan;
      else
        obj.last_plan = best_plan;
      end

      [ts, coefs] = QPReactiveRecoveryPlan.swingTraj(best_plan, foot_state.(best_plan.foot));
      pp = mkpp(ts, coefs, 6);

      ts = linspace(pp.breaks(1), pp.breaks(end));
      obj.lcmgl.glColor3f(1.0, 0.2, 0.2);
      obj.lcmgl.glLineWidth(1);
      obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
      for j = 1:length(ts)-1
        p = ppval(pp, ts(j));
        obj.lcmgl.glVertex3f(p(1), p(2), p(3));
        p = ppval(pp, ts(j+1));
        obj.lcmgl.glVertex3f(p(1), p(2), p(3));
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

      if strcmp(best_plan.foot, 'right')
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
      qp_input.body_motion_data = struct('body_id', obj.robot.foot_body_id.(best_plan.foot),...
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
    function [y, y_is_in_convhull] = closestPointInConvexHull(x, V)
      if size(V, 1) > 3 || size(V, 2) > 8
        error('V is too large for our custom solver')
      end

      u = iris.least_distance.cvxgen_ldp(bsxfun(@minus, V, x));
      y_is_in_convhull = norm(u) < 1e-3;
      y = u + x;
    end

    function intercept_plans = getInterceptPlans(foot_states, swing_foot, foot_vertices, r_ic, u, omega)
      stance_foot = obj.OTHER_FOOT.(swing_foot);

      % Find the center of pressure, which we'll place as close as possible to the ICP
      r_cop = QPReactiveRecoveryPlan.closestPointInConvexHull(r_ic, foot_vertices.(stance_foot));

      % Now transform the problem so that the x axis is aligned with (r_ic - r_cop)
      xprime = r_ic - r_cop / norm(r_ic - r_cop);
      yprime = [0, -1; 1, 0] * xprime;
      R = [xprime'; yprime'];
      foot_states_prime = foot_states;
      foot_vertices_prime = foot_vertices;
      for f = fieldnames(foot_states)'
        foot = f{1};
        foot_states_prime.(foot).position(1:2) = R * (foot_states.(foot).position(1:2) - r_cop);
        foot_states_prime.(foot).velocity(1:2) = R * foot_state.(foot).velocity(1:2);
        foot_vertices_prime.(foot) = R * foot_vertices_prime.(foot)
      end
      r_ic_prime = R * (r_ic - r_cop);
      assert(abs(r_ic_prime(2)) < 1e-6);

      intercept_plans = QPReactiveRecoveryPlan.getLocalFrameIntercepts(foot_states_prime, swing_foot, foot_vertices_prime, r_ic_prime, u, omega);
    end

    function intercept_plans = getLocalFrameIntercepts(foot_states, swing_foot, foot_vertices, r_ic_prime, u, omega)
    end

    function intercepts = bangbang(x0, xd0, xf, u_max)
      % xf = x0 + 1/2 xd0 tf + 1/4 u tf^2 - 1/4 xd0^2 / u
      % 1/4 u tf^2 + 1/2 xd0 tf + x0 - xf - 1/4 xd0^2 / u = 0
      % a = 1/4 u
      % b = 1/2 xd0
      % c = x0 - xf - 1/4 xd0^2 / u
      intercepts = struct('tf', {}, 'tswitch', {}, 'u', {});

      figure(7)
      clf
      hold on

      for u = [u_max, -u_max]
        a = 0.25 * u;
        b = 0.5 * xd0;
        c = x0 - xf - 0.25 * xd0^2 / u;


        t_roots = [(-b + sqrt(b^2 - 4*a*c)) / (2*a), (-b - sqrt(b^2 - 4*a*c)) / (2*a)]
        mask = false(size(t_roots));
        for j = 1:numel(t_roots)
          if isreal(t_roots(j)) && t_roots(j) >= abs(xd0 / u)
            mask(j) = true;
          end
        end
        tf = unique(t_roots(mask));

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

        tt = linspace(0, 3);
        plot(tt, a*tt.^2 + b*tt + c, 'g-')
        roots([a; b; c])
        xswitch = x0 + xd0 * tswitch + 0.5 * u * tswitch.^2;
        xdswitch = xd0 + u * tswitch;

        for j = 1:numel(tswitch)
          tt = linspace(0, tswitch(j));
          plot(tt, x0 + xd0 * tt + 0.5 * u * tt.^2);

          tt = linspace(tswitch(j), tf(j));
          plot(tt, xswitch + xdswitch * (tt - tswitch(j)) + 0.5 * -1 * u * (tt - tswitch(j)).^2);

          plot(tf(j), xf, 'ro');
        end
      end
    end

    function [ts, coefs] = swingTraj(intercept_plan, foot_state)
      if intercept_plan.t_switch > 0
        sizecheck(intercept_plan.r_foot_new, [6, 1]);
        swing_height = 0.05;
        slack = 10;
        % Plan a one- or two-piece polynomial spline to get to intercept_plan.r_foot_new with final velocity 0. 
        params = struct('r0', foot_state.position,...
                        'rd0',foot_state.velocity + [0;0;0.1;0;0;0],...
                        'rf',intercept_plan.r_foot_new,...
                        'rdf',[0;0;-0.1;0;0;0],...
                        'r_switch', [foot_state.position(1:2);
                                     intercept_plan.r_foot_new(3) + swing_height;
                                     foot_state.position(4:6)],...
                        'r_switch_slack', [slack; slack; 0; slack; slack; slack],... 
                        'rd_switch', zeros(6, 1),...
                        'rd_switch_slack', [slack; slack; 0; slack; slack; slack],...
                        't_switch', intercept_plan.t_switch,...
                        't_f', intercept_plan.t_intercept);
        settings = struct('verbose', 0);
        [vars, status] = freeSplineMex(params, settings);
        coefs = [cat(3, vars.C1_3, vars.C1_2, vars.C1_1, vars.C1_0), cat(3, vars.C2_3, vars.C2_2, vars.C2_1, vars.C2_0)];
        ts = [0, intercept_plan.t_switch, intercept_plan.t_intercept];
      else
        ts = [0, intercept_plan.t_intercept];
        coefs = cubicSplineCoefficients(intercept_plan.t_intercept, foot_state.position, intercept_plan.r_foot_new, foot_state.velocity, zeros(6,1));
      end

      % pp = mkpp(ts, coefs, 6);

      % tt = linspace(0, intercept_plan.t_intercept);
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

      intercept_plans = struct('foot', {}, 'r_cop', {}, 'u', {}, 't_intercept', {}, 't_switch', {}, 'error', {}, 'r_foot_new', {}, 'r_ic_new', {});
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
          [t_int, t_switch, r_foot_new, r_ic_new] = QPReactiveRecoveryPlan.icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, available_u(k), offset);
          
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
                                            't_intercept', t_int(i),...
                                            't_switch', t_switch(i),...
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

      tt = linspace(0, max([t_int, 2]));

      % figure(1)
      % clf
      % hold on
      % plot(tt, polyval(p, tt), 'g--');
      % plot(tt, a.*exp(b.*tt) + c, 'g-');
      % plot(tt, polyval(p_spline, tt), 'r-');
      % plot(t_int, polyval(p_spline, t_int), 'ro');
    end

    function [t_int, t_switch, r_foot_new, r_ic_new] = icpIntercept(r_ic, r_cop, omega, r_foot, rd_foot, u, offset)
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
      t_switch = 0.5 * (t_int - ld_foot / u);

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


