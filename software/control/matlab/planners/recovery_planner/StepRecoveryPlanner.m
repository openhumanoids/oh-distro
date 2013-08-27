classdef StepRecoveryPlanner < DRCPlanner
  properties
    biped
    last_contact_state = struct('right_contact', false, 'left_contact', false)
  end

  methods
    function obj = StepRecoveryPlanner(biped)
      obj = obj@DRCPlanner();
      obj.biped = biped;
      obj = addInput(obj, 'foot_contact', 'FOOT_CONTACT_ESTIMATE', 'drc.foot_contact_estimate_t', 1, 1, 0);
      obj = addInput(obj, 'x0', 'EST_ROBOT_STATE', obj.biped.getStateFrame().lcmcoder, 1, 1, 0);
      obj = addInput(obj, 'stop_walking', 'STOP_WALKING', 'drc.plan_control_t', 1, 1, 1);
    end


    function plan(obj, data)
      persistent has_run
      if isempty(has_run)
        has_run = true;
      else
        return
      end
      if data.foot_contact.left_contact || data.foot_contact.right_contact
        obj.last_contact_state = data.foot_contact; % remember the last foot contact in case both feet are in the air while we're falling 
      end

      nq = getNumDOF(obj.biped);
      q = data.x0(1:nq);
      qd = data.x0(nq+(1:nq));
      kinsol = doKinematics(obj.biped, q, false, true, qd);

      limp_base = [];
      if obj.last_contact_state.right_contact
        limp_base(:,end+1) = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(1), obj.biped.foot_contact_offsets.right.center, false);
      end
      if obj.last_contact_state.left_contact
        limp_base(:,end+1) = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(2), obj.biped.foot_contact_offsets.left.center, false);
      end
      limp_base = mean(limp_base, 2);

      [xcom, J] = getCOM(obj.biped, kinsol);
      xlimp = [xcom(1:2); J*qd];
      g = 9.81;
      dt = 0.5; % step duration, seconds
      omega_0 = sqrt(g / (xcom(3)-limp_base(3)));


      r_0 = xlimp(1:2);
      rd_0 = xlimp(3:4);
      % if obj.last_contact_state.right_contact && obj.last_contact_state.left_contact
        %% choose a foot to step with
      if obj.last_contact_state.right_contact
        is_right_foot = true;
      else
        is_right_foot = false;
      end

      rpos = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(1), obj.biped.foot_contact_offsets.right.center, 1);
      lpos = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(2), obj.biped.foot_contact_offsets.left.center, 1);
      if is_right_foot
        X = [lpos, rpos];
      else
        X = [rpos, lpos];
      end
      footsteps = struct('pos', X(:,1), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', zeros(6,1), 'is_right_foot', ~is_right_foot, 'is_in_contact', false);
      footsteps(end+1) = struct('pos', X(:,2), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', zeros(6,1), 'is_right_foot', is_right_foot, 'is_in_contact', true);

      r_ic0 = r_0(1:2) + rd_0(1:2) / omega_0;
      while 1
        % r_a0 = obj.biped.footOrig2Contact(X(:,end), 'center', is_right_foot);
        r_a0 = X(:,end);
        R = rotmat(r_a0(6));
        if is_right_foot
          r_a1_des = r_ic0 * exp(dt*omega_0) + r_a0(1:2) * (1 - exp(dt*omega_0)) + exp(-dt*omega_0) * R * [0; obj.biped.nom_step_width / 2];
        else
          r_a1_des = r_ic0 * exp(dt*omega_0) + r_a0(1:2) * (1 - exp(dt*omega_0)) + exp(-dt*omega_0) * R * [0; -obj.biped.nom_step_width / 2];
        end
        M = rotmat(-r_a0(6));
        des_rel = [M * (r_a1_des - r_a0(1:2)); zeros(4,1)];
        % desired = fitStepToTerrain(obj.biped, [desired; r_a0(3:6)], 'center'); 
        [A, b] = getFootstepLinearCons(obj.biped, is_right_foot, struct('max_step_width', 1.5*obj.biped.max_step_width, 'backward_step', obj.biped.max_forward_step, 'forward_step', obj.biped.max_forward_step));

        x = quadprog(diag([1,1,1,1,1,1]), zeros(6,1), A, b - A * des_rel,[],[],[-inf,-inf,0,0,0,0],[inf,inf,0,0,0,0],[],optimset('Algorithm', 'interior-point-convex'));
        r_a1_rel = des_rel + x;
        r_a1 = [rotmat(r_a0(6)) * r_a1_rel(1:2) + r_a0(1:2); r_a0(3:6)];

        % r_a1 = fmincon(@(x) norm(x-b), b, [],[],[],[],r_a0(1:2)-[2;2], r_a0(1:2)+[2;2],@(r_a1) deal(checkStepReach(obj.biped, [r_a0], [r_a1;r_a0(3:6)], is_right_foot, struct('max_step_width', 1.5*obj.biped.max_step_width)),[]),optimset('Algorithm','interior-point'));
        % X(:,end+1) = fitStepToTerrain(obj.biped, [r_a1; r_a0(3:6)], 'center');
        X(:,end+1) = fitStepToTerrain(obj.biped, r_a1, 'center');
        % X(:,end+1) = fitStepToTerrain(obj.biped, obj.biped.footContact2Orig([r_a1; r_a0(3:6)], 'center', ~is_right_foot));

        corners = [obj.biped.footOrig2Contact(X(:,end-1), 'toe', is_right_foot),...
                   obj.biped.footOrig2Contact(X(:,end-1), 'heel', is_right_foot),...
                   obj.biped.footOrig2Contact(X(:,end), 'toe', ~is_right_foot),...
                   obj.biped.footOrig2Contact(X(:,end), 'heel', ~is_right_foot)];
        r_ic1 = (r_ic0 - r_a0(1:2)) * exp(dt*omega_0) + r_a0(1:2);
        r_ic2 = (r_ic1 - r_a1(1:2)) * exp(dt*omega_0) + r_a1(1:2);
        final_ok = inpolygon(r_ic1(1), r_ic1(2), corners(1,:), corners(2,:));
        footsteps(end+1) = struct('pos', X(:,end), 'step_speed', 0, 'step_height', 0, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', ~is_right_foot, 'is_in_contact', true);

        plot_lcm_points([r_ic0', limp_base(3)], [1,0,0], 250, 'r_ic0', 1, 1);
        plot_lcm_points([r_a0(1:3)'], [1,1,0], 253, 'r_a0', 1,1);
        r_ic1 = (r_ic0 - r_a0(1:2)) * exp(dt*omega_0) + r_a0(1:2);
        plot_lcm_points([r_ic1', r_a0(3)], [1,.5,0], 252, 'r_ic1', 1, 1);
        plot_lcm_points([r_a1_des', r_a0(3)], [0,0,1], 255, 'desired', 1, 1);
        plot_lcm_points([r_a1', r_a0(3)], [0,1,0], 251, 'r_a1', 1, 1);
        r_ic2 = (r_ic1 - r_a1(1:2)) * exp(dt*omega_0) + r_a1(1:2);
        plot_lcm_points([r_ic2', r_a0(3)], [1,.8,0], 260, 'r_ic2', 1, 1);
        if final_ok || size(X,2) > 5
          break
        end
        r_ic0 = r_ic1;
        is_right_foot = ~is_right_foot;
      end
      plot_lcm_poses(X(1:3,1:2:size(X,2))', X(6:-1:4,1:2:size(X,2))', 256, 'Odd', 1, true, false, 257);
      plot_lcm_poses(X(1:3,2:2:size(X,2))', X(6:-1:4,2:2:size(X,2))', 258, 'Even', 1, true, false, 258);

      for j = 1:length(footsteps)
        footsteps(j).id = obj.biped.getNextStepID();
        footsteps(j).step_speed = -dt; % negative speed signals a fixed duration
        footsteps(j).step_height = 0.05;
        footsteps(j).pos = obj.biped.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
      end
      footstep_opts = struct('ignore_terrain', 1, 'mu', 1);
      obj.biped.publish_footstep_plan(footsteps, etime(clock,[1970 1 1 0 0 0])*1000000, 1, footstep_opts);
      [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(obj.biped, data.x0, footsteps, footstep_opts);
      link_constraints = buildLinkConstraints(obj.biped, q, foottraj, []);
      s1dot = fnder(V.s1,1);
      s2dot = fnder(V.s2,1);
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      xstar = d.xstar;     
      qstar = xstar(1:nq);
      walking_plan = struct('S',V.S,'s1',V.s1,'s2',V.s2,'s1dot',s1dot,'s2dot',s2dot,...
          'support_times',support_times,'supports',{supports},'comtraj',comtraj,'mu',footstep_opts.mu,'t_offset',0,...
          'link_constraints',link_constraints,'zmptraj',zmptraj,'qtraj',qstar,'ignore_terrain',footstep_opts.ignore_terrain)
      walking_pub = WalkingPlanPublisher('WALKING_PLAN');
      walking_pub.publish(0,walking_plan);
    end
  end
end
