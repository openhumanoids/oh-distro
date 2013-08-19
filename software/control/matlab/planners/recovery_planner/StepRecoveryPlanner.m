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
      obj = addInput(obj, 'x0', 'EST_ROBOT_STATE', obj.biped.getStateFrame().lcmcoder, 1, 1, 1);
    end


    function plan(obj, data)
      % if data.foot_contact.left_contact == obj.last_contact_state.left_contact && data.foot_contact.right_contact == obj.last_contact_state.right_contact
      %   return;
      % end
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
      dt = 0.3; % step duration, seconds
      omega_0 = sqrt(g / (xcom(3)-limp_base(3)));
      capture_pt = xlimp(1:2) + 1/omega_0 * xlimp(3:4);

      plot_lcm_points([capture_pt', limp_base(3)], [1,0,0], 250, 'r_ic0', 1, 1);

      r_0 = xlimp(1:2);
      rd_0 = xlimp(3:4);
      r_ic0 = capture_pt;
      % if obj.last_contact_state.right_contact && obj.last_contact_state.left_contact
        %% choose a foot to step with
      if obj.last_contact_state.right_contact
        %% step with left first
        r_a0 = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(1), obj.biped.foot_contact_offsets.right.center, 1);
        R = rotmat(r_a0(6));
        b = r_0(1:2) * exp(dt*omega_0) + 1/omega_0 * rd_0(1:2) * exp(dt*omega_0) + r_a0(1:2) * (1 - exp(dt*omega_0)) + exp(-dt*omega_0) * R * [0; obj.biped.nom_step_width / 2];
        r_a1 = fmincon(@(x) norm(x-b), b, [],[],[],[],r_a0(1:2)-[2;2], r_a0(1:2)+[2;2],@(r_a1) deal(checkStepFeasibility(obj.biped, [r_a0], [r_a1;r_a0(3:6)], 1, struct('max_step_width', 1.5*obj.biped.max_step_width)),[]),optimset('Algorithm','interior-point'));
      else
        r_a0 = forwardKin(obj.biped, kinsol, obj.biped.foot_bodies_idx(2), obj.biped.foot_contact_offsets.left.center, 1);
        R = rotmat(r_a0(6));
        b = r_0(1:2) * exp(dt*omega_0) + 1/omega_0 * rd_0(1:2) * exp(dt*omega_0) + r_a0(1:2) * (1 - exp(dt*omega_0)) + exp(-dt*omega_0) * R * [0; -obj.biped.nom_step_width / 2];
        r_a1 = fmincon(@(x) norm(x-b), b, [],[],[],[],r_a0(1:2)-[2;2], r_a0(1:2)+[2;2],@(r_a1) deal(checkStepFeasibility(obj.biped, [r_a0], [r_a1;r_a0(3:6)], 0, struct('max_step_width', 1.5*obj.biped.max_step_width)),[]),optimset('Algorithm','interior-point'));
        %% step with right first
      end
      plot_lcm_points([r_a0(1:3)'], [1,1,0], 253, 'r_a0', 1,1);
      r_ic1 = (r_ic0 - r_a0(1:2)) * exp(dt*omega_0) + r_a0(1:2);
      plot_lcm_points([r_ic1', r_a0(3)], [1,.5,0], 252, 'r_ic1', 1, 1);
      % r_a1 = b;
      plot_lcm_points([b', r_a0(3)], [0,0,1], 255, 'b', 1, 1);
      plot_lcm_points([r_a1', r_a0(3)], [0,1,0], 251, 'r_a1', 1, 1);
      r_ic2 = (r_ic1 - r_a1(1:2)) * exp(dt*omega_0) + r_a1(1:2);
      plot_lcm_points([r_ic2', r_a0(3)], [1,.8,0], 254, 'r_ic2', 1, 1);

    end
  end
end
