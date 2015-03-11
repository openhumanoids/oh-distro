classdef StatelessWalkingPlanner
  methods
    function obj = StatelessWalkingPlanner()
      % warm up the LQR function. The first call to lqr() in matlab 2014a is 
      % ludicrously slow (by a factor of 100, generally), but subsequent calls 
      % are all fast. We can save time by solving a trivial LQR problem first,
      % so that our later calls to lqr() will be faster. 
      lqr(1,1,1,1,0);
    end
  end

  methods(Static=true)
    function walking_plan = plan_walking(r, request, compute_xtraj, simulate)
      if nargin < 4
        simulate = false;
      end
      if simulate
        compute_xtraj = true;
      end
      debug = false;

      x0 = r.getStateFrame().lcmcoder.decode(request.initial_state);
      r = r.setInitialState(x0);
      q0 = x0(1:end/2);
      nq = getNumPositions(r);

      if request.use_new_nominal_state
        xstar = r.getStateFrame().lcmcoder.decode(request.new_nominal_state);
      else
        xstar = r.loadFixedPoint();
      end

      r = configureDRCTerrain(r, request.footstep_plan.params.map_mode, q0);

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
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkId('r_hand+r_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end
      if request.fix_left_hand
        walking_plan_data = walking_plan_data.fix_link(r, kinsol, r.findLinkId('l_hand+l_hand_point_mass'), [0; 0.1; 0], 0.05, 0);
      end

      % Convert the walking plan data to the DRC type that extends
      % in terms of LCM transcription methods
      walking_ctrl_data = DRCQPWalkingPlan.from_drake_walking_data(walking_plan_data, r);
      ts = linspace(0,walking_ctrl_data.comtraj.tspan(2),150);

      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'walking-plan');

      for i=1:length(ts)
%         lcmgl.glColor3f(0, 0, 1);
%         lcmgl.sphere([walking_plan.comtraj.eval(ts(i));0], 0.01, 20, 20);
        lcmgl.glColor3f(0, 1, 0);
        lcmgl.sphere([walking_ctrl_data.zmptraj.eval(ts(i));0], 0.01, 20, 20);
      end
      lcmgl.switchBuffers();

      if compute_xtraj
        [xtraj, ~, ts] = planWalkingStateTraj(r, walking_plan_data, xstar);
        joint_names = r.getStateFrame.coordinates(1:getNumPositions(r));
        joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
        walking_plan = WalkingPlan(ts, xtraj, joint_names);

        if simulate
          x0_resolved = r.resolveConstraints(r.getInitialState());
          r = r.setInitialState(x0_resolved);
          traj = atlasUtil.simulateWalking(r, walking_ctrl_data, 1, false, false, false, false);
          ts = traj.getBreaks();
          if length(ts) > 300
            % Limit the number of samples in the plan to get around LCM packet size issues in Java/Matlab
            ts = linspace(ts(1), ts(end), 300);
          end
          xs = traj.eval(ts);
          walking_plan = WalkingPlan(ts, xs, joint_names);
        end
      else
        walking_plan = walking_ctrl_data;
      end
      disp('done')
    end
  end
end
