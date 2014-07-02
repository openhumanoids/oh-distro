classdef StatelessWalkingPlanner
  methods
    function obj = StatelessWalkingPlanner()
    end
  end

  methods(Static=true)
    function walking_plan = plan_walking(r, request, compute_xtraj)
      debug = false;

      x0 = r.getStateFrame().lcmcoder.decode(request.initial_state);
      q0 = x0(1:end/2);
      nq = getNumDOF(r);

      if request.use_new_nominal_state
        xstar = r.getStateFrame().lcmcoder.decode(request.new_nominal_state);
      else
        xstar = r.loadFixedPoint();
      end

      r = r.configureDRCTerrain(request.footstep_plan.params.map_mode, q0);

      qstar = xstar(1:nq);

      footstep_plan = FootstepPlan.from_footstep_plan_t(request.footstep_plan, r);

      % Align the first two steps to the current feet poses
      feet_centers = feetPosition(r, q0);
      if footstep_plan.footsteps(1).frame_id == r.foot_frame_id.right
        footstep_plan.footsteps(1).pos = feet_centers.right;
        footstep_plan.footsteps(2).pos = feet_centers.left;
      else
        footstep_plan.footsteps(1).pos = feet_centers.left;
        footstep_plan.footsteps(2).pos = feet_centers.right;
      end

      % Slow down the first and last steps, if necessary
      for j = [length(footstep_plan.footsteps)-1,length(footstep_plan.footsteps)]
        footstep_plan.footsteps(j).walking_params.step_speed = min([footstep_plan.footsteps(j).walking_params.step_speed, 1.5]);
      end

      % Compute the ZMP traj, COM traj, feet trajs, etc.
      walking_plan_data = planWalkingZMP(r, x0, footstep_plan);

      if request.fix_right_hand
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkInd('r_hand+r_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end
      if request.fix_left_hand
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkInd('l_hand+l_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end

      if compute_xtraj
        [xtraj, ~, ts] = planWalkingStateTraj(r, walking_plan_data, xstar);
        joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
        joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'

        walking_plan = WalkingPlan(ts, xtraj, joint_names);
      else
        walking_plan = WalkingControllerData.from_drake_walking_data(walking_plan_data, qstar);
      end
      disp('done')
    end
  end
end
