classdef StepRecoveryPlanner < DRCPlanner
  properties
    biped
  end

  methods
    function obj = StepRecoveryPlanner(biped)
      obj = obj@DRCPlanner();
      obj.biped = biped;
      obj = addInput(obj, 'x0', 'EST_ROBOT_STATE', obj.biped.getStateFrame().lcmcoder, 1, 1, 1);
      obj = addInput(obj, 'foot_contact', 'FOOT_CONTACT_ESTIMATE', 'drc.foot_contact_estimate_t', 1, 1, 0);
      % obj = addInput(obj, 'stop_walking', 'BRACE_FOR_FALL', 'drc.utime_t', 0, 1, 1);
      % obj = addInput(obj, 'push', 'ATLAS_PUSH', 'drc.atlas_push_t', 0,1,1);
    end


    function plan(obj, data)
      % profile on
      planning_time = getLastTimestamp(obj, 1)/1e6;
      if data.foot_contact.left_contact || data.foot_contact.right_contact
        contact_state = data.foot_contact;
      else
        contact_state = struct('right_contact', true, 'left_contact', true);
      end

      footsteps = recoverySteps(obj.biped, x0, contact_state);




      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

      first_step = true;

      while 1
        r_a0 = footsteps(end).pos;
        r_ic1 = (r_ic0 - r_a0(1:2)) * exp(dts(1)*omega_0) + r_a0(1:2);

        if first_step
          r_a1 = footsteps(end-1).pos;
          r_a1_des = r_a1;
          first_step = false;
        else
          R = rotmat(r_a0(6));
          if is_right_foot
            r_a1_des = r_ic0 * exp(dts(1)*omega_0) + r_a0(1:2) * (1 - exp(dts(1)*omega_0)) + exp(-dts(2)*omega_0) * R * [0; obj.biped.nom_step_width / 2];
          else
            r_a1_des = r_ic0 * exp(dts(1)*omega_0) + r_a0(1:2) * (1 - exp(dts(1)*omega_0)) + exp(-dts(2)*omega_0) * R * [0; -obj.biped.nom_step_width / 2];
          end
          M = rotmat(-r_a0(6));
          des_rel = [M * (r_a1_des - r_a0(1:2)); zeros(4,1)];
          [A, b] = getFootstepLinearCons(obj.biped, is_right_foot, struct('max_step_width', 1.5*obj.biped.max_step_width, 'backward_step', obj.biped.max_forward_step, 'forward_step', obj.biped.max_forward_step));
          model.A = sparse(A);
          model.obj = zeros(6,1);
          model.sense = '<';
          model.rhs = b - A * des_rel;
          model.lb = [-inf,-inf,0,0,0,0]';
          model.ub = [inf,inf,0,0,0,0]';
          model.objcon = 0;
          model.Q = sparse(diag(ones(1,6)));

          params.outputflag = 1;
          result = gurobi(model, params);
          x = result.x;

          r_a1_rel = des_rel + x;
          r_a1 = [rotmat(r_a0(6)) * r_a1_rel(1:2) + r_a0(1:2); r_a0(3:6)];
        end

        r_ic2 = (r_ic1 - r_a1(1:2)) * exp(dts(2)*omega_0) + r_a1(1:2);

        footsteps(end+1) = struct('pos', fitStepToTerrain(obj.biped, r_a1, 'center'), 'step_speed', -dts(2), 'step_height', 0.05, 'id', 0, 'pos_fixed', ones(6,1), 'is_right_foot', ~is_right_foot, 'is_in_contact', true);

        plot_lcm_points([r_ic0', limp_base(3)], [1,0,0], 250, 'r_ic0', 1, 1);
        plot_lcm_points(r_a0(1:3)', [1,1,0], 253, 'r_a0', 1,1);
        plot_lcm_points([r_ic1', r_a0(3)], [1,.5,0], 252, 'r_ic1', 1, 1);
        plot_lcm_points([r_a1_des', r_a0(3)], [0,0,1], 255, 'desired', 1, 1);
        plot_lcm_points([r_a1', r_a0(3)], [0,1,0], 251, 'r_a1', 1, 1);
        plot_lcm_points([r_ic2', r_a0(3)], [1,.8,0], 260, 'r_ic2', 1, 1);

        corners = [obj.biped.footOrig2Contact(footsteps(end-1).pos, 'toe', is_right_foot),...
                   obj.biped.footOrig2Contact(footsteps(end-1).pos, 'heel', is_right_foot),...
                   obj.biped.footOrig2Contact(footsteps(end).pos, 'toe', ~is_right_foot),...
                   obj.biped.footOrig2Contact(footsteps(end).pos, 'heel', ~is_right_foot)];
        final_ok = inpolygon(r_ic1(1), r_ic1(2), corners(1,:), corners(2,:));
        if final_ok || size(X,2) > 5
          break
        end
        r_ic0 = r_ic1;
        dts(1) = dts(2);
        is_right_foot = ~is_right_foot;
      end
      plot_lcm_poses(X(1:3,1:2:size(X,2))', X(6:-1:4,1:2:size(X,2))', 256, 'Odd', 1, true, false, 257);
      plot_lcm_poses(X(1:3,2:2:size(X,2))', X(6:-1:4,2:2:size(X,2))', 258, 'Even', 1, true, false, 258);

      for j = 1:length(footsteps)
        footsteps(j).id = obj.biped.getNextStepID();
        footsteps(j).pos = obj.biped.footContact2Orig(footsteps(j).pos, 'center', footsteps(j).is_right_foot);
      end
      footstep_opts = struct('ignore_terrain', 1, 'mu', 1, 'behavior', drc.footstep_opts_t.BEHAVIOR_WALKING);
      % obj.biped.publish_footstep_plan(footsteps, etime(clock,[1970 1 1 0 0 0])*1000000, 1, footstep_opts);
      [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(obj.biped, data.x0, footsteps, footstep_opts);
      s1dot = fnder(V.s1,1);
      s2dot = fnder(V.s2,1);
      link_constraints = buildLinkConstraints(obj.biped, q, foottraj, []);
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      xstar = d.xstar;     
      qstar = xstar(1:nq);
      walking_plan = struct('S',V.S,'s1',V.s1,'s2',V.s2,'s1dot',s1dot,'s2dot',s2dot,...
          'support_times',support_times,'supports',{supports},'comtraj',comtraj,'mu',footstep_opts.mu,'t_offset',planning_time,...
          'link_constraints',link_constraints,'zmptraj',zmptraj,'qtraj',qstar,'ignore_terrain',footstep_opts.ignore_terrain);
      walking_pub = WalkingPlanPublisher('WALKING_PLAN');
      walking_pub.publish(planning_time*1e6,walking_plan);
      % profile viewer
    end
  end
end
